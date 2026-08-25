/**************************************************************************/
/*  jolt_rope_3d.cpp                                                      */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "jolt_rope_3d.h"

#include "../misc/jolt_type_conversions.h"
#include "../spaces/jolt_broad_phase_layer.h"
#include "../spaces/jolt_query_collectors.h"
#include "../spaces/jolt_space_3d.h"
#include "jolt_area_3d.h"
#include "jolt_body_3d.h"
#include "jolt_object_3d.h"

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace {

constexpr float ROPE_EPSILON = 1e-6f;

// Contacts kept per segment per frame. A segment wedged into a corner needs more than one plane to
// resolve correctly, but past a handful the extra planes are redundant and only cost substep time.
constexpr int ROPE_MAX_CONTACTS_PER_SEGMENT = 4;

// How far beyond touching the narrow phase still reports a contact. Detection runs once per frame
// while resolution runs every substep, so contacts must be gathered slightly early: otherwise a
// particle that moves into a surface part-way through the frame would find no plane to resolve
// against and would pass straight through it.
constexpr float ROPE_SPECULATIVE_MARGIN = 0.02f;

// Ratio of a cable's along-axis drag coefficient to its crosswise one. Air resistance on a cylinder
// is dominated by the component across its axis; sliding along its own length is nearly free by
// comparison. Roughly 0.02 for a real cable, and the reason a slack rope flicks sideways and settles
// rather than gliding through the air like a bead on a wire.
constexpr float ROPE_TANGENTIAL_DRAG_RATIO = 0.02f;

// Twist is skipped entirely at or above this compliance, which is the default: a rope that offers no
// torsional resistance is what the simulation did before twist existed, so leaving it off costs
// nothing and changes nothing for scenes that were built without it.
constexpr float ROPE_TWIST_DISABLED_COMPLIANCE = 1e6f;

// Bending is skipped entirely at or above this compliance. A rope is floppy by default and the
// constraint would just be arithmetic with no visible effect.
constexpr float ROPE_BEND_DISABLED_COMPLIANCE = 1e6f;

// Fraction of an accumulated positional correction that is handed back to a rigid body as a
// reaction. This is the pin and the contacts -- the constraints that transmit force through the rope
// itself -- and it stays gentle: the gap it measures is re-derived every substep against a body that
// cannot actually move, so it carries solver noise, and feeding that back at full strength turns
// into a torque that quietly drains a freely rotating load (measured at 18.6 % of a swing's energy
// over fifteen seconds).
constexpr float ROPE_REACTION_RELAXATION = 0.25f;

// Ceiling on how fast the limit joint may reel a body back in. Only reachable when a body is already
// well outside its limit -- teleported there, or spawned there -- because the limit is solved
// predictively and so does not normally let an error build up at all. See `_solve_velocities()`.
constexpr float ROPE_MAX_REEL_IN_SPEED = 1.0f;

// Gauss-Seidel sweeps the bending constraint gets per substep. It gets more than the distance
// constraint because it is the constraint that has to carry a shape along the whole rope, and
// Gauss-Seidel propagates about one particle per sweep; measured on a two-metre rope, going from two
// sweeps to eight moves the stiffest setting from barely distinguishable from floppy to close to the
// tightest arc the length constraints allow. It costs nothing at the default `bend_compliance`,
// where `_solve_bending()` returns immediately.
constexpr int ROPE_BEND_SWEEPS = 8;

// Rope-specific query filter. `JoltQueryFilter3D` is built around `PhysicsDirectSpaceState3D` query
// parameters, so ropes carry their own trivial version over layer/mask plus the exception set.
class JoltRopeQueryFilter3D final
		: public JPH::BroadPhaseLayerFilter,
		  public JPH::ObjectLayerFilter,
		  public JPH::BodyFilter {
	const JoltSpace3D &space;
	const VSet<RID> &excluded;
	uint32_t collision_mask = 0;

public:
	JoltRopeQueryFilter3D(const JoltSpace3D &p_space, uint32_t p_collision_mask, const VSet<RID> &p_excluded) :
			space(p_space), excluded(p_excluded), collision_mask(p_collision_mask) {}

	virtual bool ShouldCollide(JPH::BroadPhaseLayer p_broad_phase_layer) const override {
		const JPH::BroadPhaseLayer::Type broad_phase_layer = (JPH::BroadPhaseLayer::Type)p_broad_phase_layer;

		switch (broad_phase_layer) {
			case (JPH::BroadPhaseLayer::Type)JoltBroadPhaseLayer::BODY_STATIC:
			case (JPH::BroadPhaseLayer::Type)JoltBroadPhaseLayer::BODY_STATIC_BIG:
			case (JPH::BroadPhaseLayer::Type)JoltBroadPhaseLayer::BODY_DYNAMIC: {
				return true;
			} break;
			default: {
				// Ropes never collide with areas.
				return false;
			}
		}
	}

	virtual bool ShouldCollide(JPH::ObjectLayer p_object_layer) const override {
		JPH::BroadPhaseLayer object_broad_phase_layer = JoltBroadPhaseLayer::BODY_STATIC;
		uint32_t object_collision_layer = 0;
		uint32_t object_collision_mask = 0;

		space.map_from_object_layer(p_object_layer, object_broad_phase_layer, object_collision_layer, object_collision_mask);

		return (collision_mask & object_collision_layer) != 0;
	}

	virtual bool ShouldCollide(const JPH::BodyID &p_body_id) const override { return true; }

	virtual bool ShouldCollideLocked(const JPH::Body &p_body) const override {
		if (excluded.is_empty()) {
			return true;
		}

		const JoltObject3D *object = reinterpret_cast<const JoltObject3D *>(p_body.GetUserData());
		return object == nullptr || !excluded.has(object->get_rid());
	}
};

// Orthonormal basis whose +Y axis is `p_axis`, matching the axis convention of `JPH::CapsuleShape`.
//
// The handedness matters: Jolt is handed a rotation matrix, and a mirrored basis makes the query
// silently return nothing. `z` must therefore be `x cross y`, not `y cross x`.
Basis capsule_basis_for_axis(const Vector3 &p_axis) {
	const Vector3 reference = Math::abs(p_axis.y) > 0.99 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
	const Vector3 x = reference.cross(p_axis).normalized();
	const Vector3 z = x.cross(p_axis);

	Basis basis;
	basis.set_column(0, x);
	basis.set_column(1, p_axis);
	basis.set_column(2, z);
	return basis;
}

} // namespace

JoltRope3D::JoltRope3D() :
		active_list(this) {
}

JoltRope3D::~JoltRope3D() {
	if (space != nullptr) {
		set_space(nullptr);
	}
}

void JoltRope3D::set_space(JoltSpace3D *p_space) {
	if (space == p_space) {
		return;
	}

	if (space != nullptr) {
		// Constraints belong to the space that holds them, so they cannot outlive this one.
		_release_all_limit_constraints();
		space->dequeue_rope(&active_list);
	}

	space = p_space;

	colliders.clear();
	contacts.clear();
	collider_lookup.clear();

	if (space != nullptr) {
		space->enqueue_rope(&active_list);
	}
}

/* -------------------------------------------------------------------------- */
/* Topology                                                                   */
/* -------------------------------------------------------------------------- */

void JoltRope3D::set_points(const Vector<Vector3> &p_points) {
	const int count = p_points.size();

	positions.resize(count);
	prev_positions.resize(count);
	velocities.resize(count);
	inv_masses.resize(count);
	base_inv_masses.resize(count);

	const Vector3 *src = p_points.ptr();
	for (int i = 0; i < count; i++) {
		positions[i] = src[i];
		prev_positions[i] = src[i];
		velocities[i] = Vector3();
	}

	_update_rest_lengths();

	// Drop attachments that no longer have a particle to refer to.
	LocalVector<int> stale;
	for (const KeyValue<int, Attachment> &E : attachments) {
		if (E.key >= count) {
			stale.push_back(E.key);
		}
	}
	for (int index : stale) {
		if (Attachment *attachment = attachments.getptr(index)) {
			_release_limit_constraint(*attachment);
		}
		attachments.erase(index);
	}

	_update_masses();
	lra_dirty = true;
	_update_bounds();
}

Vector<Vector3> JoltRope3D::get_points() const {
	Vector<Vector3> result;
	result.resize(positions.size());

	Vector3 *dst = result.ptrw();
	for (uint32_t i = 0; i < positions.size(); i++) {
		dst[i] = positions[i];
	}

	return result;
}

Vector3 JoltRope3D::get_point_position(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int)positions.size(), Vector3());
	return positions[p_index];
}

void JoltRope3D::set_point_position(int p_index, const Vector3 &p_position) {
	ERR_FAIL_INDEX(p_index, (int)positions.size());
	positions[p_index] = p_position;
	prev_positions[p_index] = p_position;
	velocities[p_index] = Vector3();
}

Vector3 JoltRope3D::get_point_velocity(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int)velocities.size(), Vector3());
	return velocities[p_index];
}

void JoltRope3D::_update_masses() {
	const uint32_t count = positions.size();
	if (count == 0) {
		return;
	}

	const float inv_particle_mass = (float)count / MAX(total_mass, 0.0001f);

	for (uint32_t i = 0; i < count; i++) {
		base_inv_masses[i] = inv_particle_mass;
		inv_masses[i] = inv_particle_mass;
	}
}

void JoltRope3D::_update_rest_lengths() {
	// A different particle count means a different set of segments, so the carried frame no longer
	// describes this rope and has to be re-seeded rather than transported. Twist is stored against
	// that frame, so it is reset with it.
	ref_frame_valid = false;

	const uint32_t new_segments = positions.size() > 0 ? positions.size() - 1 : 0;
	if (twist_angles.size() != new_segments) {
		twist_angles.resize(new_segments);
		twist_velocities.resize(new_segments);
		twist_scratch_c.resize(new_segments);
		twist_scratch_d.resize(new_segments);
		twist_predicted.resize(new_segments);

		for (uint32_t i = 0; i < new_segments; i++) {
			twist_angles[i] = 0.0f;
			twist_velocities[i] = 0.0f;
		}

		for (KeyValue<int, Attachment> &E : attachments) {
			E.value.twist_captured = false;
			E.value.direction_captured = false;
		}
	}

	const uint32_t count = positions.size();

	const uint32_t segment_count = count > 0 ? count - 1 : 0;
	rest_lengths.resize(segment_count);
	lambdas.resize(segment_count);

	float authored_total = 0.0f;
	for (uint32_t i = 0; i < segment_count; i++) {
		rest_lengths[i] = (float)positions[i].distance_to(positions[i + 1]);
		authored_total += rest_lengths[i];
	}

	// A requested length rescales the authored spacing uniformly, which keeps the relative
	// proportions of a non-uniformly sampled pose while making the total exact. Without this,
	// adding points to a rope would make the rope longer, since every segment is measured from
	// wherever the points happened to be placed.
	if (rest_length > 0.0f && authored_total > ROPE_EPSILON) {
		const float scale = rest_length / authored_total;
		for (uint32_t i = 0; i < segment_count; i++) {
			rest_lengths[i] *= scale;
		}
	}

	// Bending spans two segments. Its rest state is deliberately *straight* rather than the shape
	// measured from the authored pose: that pose is only where the rope spawns, not something it
	// should spring back to. Straight is the zero of the constraint in `_solve_bending()`, so there
	// is no per-triple rest value to store -- only the accumulated multipliers.
	const uint32_t bend_count = count > 1 ? count - 2 : 0;
	bend_lambdas.resize(bend_count);

	cumulative_rest.resize(count);
	if (count > 0) {
		cumulative_rest[0] = 0.0f;
		for (uint32_t i = 1; i < count; i++) {
			cumulative_rest[i] = cumulative_rest[i - 1] + rest_lengths[i - 1];
		}
	}

	lra_dirty = true;
}

/* -------------------------------------------------------------------------- */
/* Parameters                                                                 */
/* -------------------------------------------------------------------------- */

void JoltRope3D::set_param(PhysicsServer3D::RopeParameter p_param, float p_value) {
	switch (p_param) {
		case PhysicsServer3D::ROPE_PARAM_RADIUS: {
			radius = MAX(p_value, 0.001f);
		} break;
		case PhysicsServer3D::ROPE_PARAM_TOTAL_MASS: {
			total_mass = MAX(p_value, 0.0001f);
			_update_masses();
		} break;
		case PhysicsServer3D::ROPE_PARAM_STRETCH_COMPLIANCE: {
			stretch_compliance = MAX(p_value, 0.0f);
		} break;
		case PhysicsServer3D::ROPE_PARAM_BEND_COMPLIANCE: {
			bend_compliance = MAX(p_value, 0.0f);
		} break;
		case PhysicsServer3D::ROPE_PARAM_LINEAR_DAMPING: {
			linear_damping = MAX(p_value, 0.0f);
		} break;
		case PhysicsServer3D::ROPE_PARAM_DRAG: {
			drag = MAX(p_value, 0.0f);
		} break;
		case PhysicsServer3D::ROPE_PARAM_GRAVITY_SCALE: {
			gravity_scale = p_value;
		} break;
		case PhysicsServer3D::ROPE_PARAM_FRICTION: {
			friction = CLAMP(p_value, 0.0f, 1.0f);
		} break;
		case PhysicsServer3D::ROPE_PARAM_RESTITUTION: {
			restitution = CLAMP(p_value, 0.0f, 1.0f);
		} break;
		case PhysicsServer3D::ROPE_PARAM_MAX_REACTION_IMPULSE: {
			max_reaction_impulse = MAX(p_value, 0.0f);
		} break;
		case PhysicsServer3D::ROPE_PARAM_TWIST_COMPLIANCE: {
			twist_compliance = MAX(p_value, 0.0f);
		} break;
		case PhysicsServer3D::ROPE_PARAM_TWIST_DAMPING: {
			twist_damping = MAX(p_value, 0.0f);
		} break;
		case PhysicsServer3D::ROPE_PARAM_LENGTH: {
			const float length = MAX(p_value, 0.0f);
			if (!Math::is_equal_approx(rest_length, length)) {
				rest_length = length;
				_update_rest_lengths();
			}
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled rope parameter: '%d'. This should not happen. Please report this.", p_param));
		}
	}
}

float JoltRope3D::get_param(PhysicsServer3D::RopeParameter p_param) const {
	switch (p_param) {
		case PhysicsServer3D::ROPE_PARAM_RADIUS:
			return radius;
		case PhysicsServer3D::ROPE_PARAM_TOTAL_MASS:
			return total_mass;
		case PhysicsServer3D::ROPE_PARAM_STRETCH_COMPLIANCE:
			return stretch_compliance;
		case PhysicsServer3D::ROPE_PARAM_BEND_COMPLIANCE:
			return bend_compliance;
		case PhysicsServer3D::ROPE_PARAM_TWIST_COMPLIANCE:
			return twist_compliance;
		case PhysicsServer3D::ROPE_PARAM_TWIST_DAMPING:
			return twist_damping;
		case PhysicsServer3D::ROPE_PARAM_LINEAR_DAMPING:
			return linear_damping;
		case PhysicsServer3D::ROPE_PARAM_DRAG:
			return drag;
		case PhysicsServer3D::ROPE_PARAM_GRAVITY_SCALE:
			return gravity_scale;
		case PhysicsServer3D::ROPE_PARAM_FRICTION:
			return friction;
		case PhysicsServer3D::ROPE_PARAM_RESTITUTION:
			return restitution;
		case PhysicsServer3D::ROPE_PARAM_MAX_REACTION_IMPULSE:
			return max_reaction_impulse;
		case PhysicsServer3D::ROPE_PARAM_LENGTH:
			return rest_length;
		default: {
			ERR_FAIL_V_MSG(0.0f, vformat("Unhandled rope parameter: '%d'. This should not happen. Please report this.", p_param));
		}
	}
}

void JoltRope3D::set_flag(PhysicsServer3D::RopeFlag p_flag, bool p_enabled) {
	switch (p_flag) {
		case PhysicsServer3D::ROPE_FLAG_INEXTENSIBLE: {
			inextensible = p_enabled;
			lra_dirty = true;
		} break;
		case PhysicsServer3D::ROPE_FLAG_TWO_WAY_COUPLING: {
			two_way_coupling = p_enabled;
		} break;
		case PhysicsServer3D::ROPE_FLAG_COLLISION_ENABLED: {
			collision_enabled = p_enabled;
			if (!collision_enabled) {
				contacts.clear();
				colliders.clear();
				collider_lookup.clear();
			}
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled rope flag: '%d'. This should not happen. Please report this.", p_flag));
		}
	}
}

bool JoltRope3D::get_flag(PhysicsServer3D::RopeFlag p_flag) const {
	switch (p_flag) {
		case PhysicsServer3D::ROPE_FLAG_INEXTENSIBLE:
			return inextensible;
		case PhysicsServer3D::ROPE_FLAG_TWO_WAY_COUPLING:
			return two_way_coupling;
		case PhysicsServer3D::ROPE_FLAG_COLLISION_ENABLED:
			return collision_enabled;
		default: {
			ERR_FAIL_V_MSG(false, vformat("Unhandled rope flag: '%d'. This should not happen. Please report this.", p_flag));
		}
	}
}

void JoltRope3D::set_simulation_substeps(int p_substeps) {
	substeps = CLAMP(p_substeps, 1, 32);
}

/* -------------------------------------------------------------------------- */
/* Pins and attachments                                                       */
/* -------------------------------------------------------------------------- */

void JoltRope3D::pin_point(int p_index, bool p_pin) {
	ERR_FAIL_INDEX(p_index, (int)positions.size());

	if (p_pin) {
		Attachment attachment;
		_carry_locks(attachment, p_index, ATTACH_STATIC, RID());
		attachment.mode = ATTACH_STATIC;
		// Re-pinning an already pinned point must not teleport it back to wherever the particle
		// happens to be now; that is `set_pin_position()`'s job.
		const Attachment *existing = attachments.getptr(p_index);
		attachment.static_position = (existing != nullptr && existing->mode == ATTACH_STATIC)
				? existing->static_position
				: positions[p_index];
		attachments[p_index] = attachment;
	} else {
		const Attachment *existing = attachments.getptr(p_index);
		if (existing != nullptr && existing->mode == ATTACH_STATIC) {
			attachments.erase(p_index);
		}
	}

	lra_dirty = true;
}

bool JoltRope3D::is_point_pinned(int p_index) const {
	const Attachment *attachment = attachments.getptr(p_index);
	return attachment != nullptr && attachment->mode == ATTACH_STATIC;
}

void JoltRope3D::set_pin_position(int p_index, const Vector3 &p_position) {
	Attachment *attachment = attachments.getptr(p_index);
	ERR_FAIL_NULL_MSG(attachment, "Rope point is not pinned.");
	ERR_FAIL_COND_MSG(attachment->mode != ATTACH_STATIC, "Rope point is attached to a body, not pinned to a position.");

	attachment->static_position = p_position;
}

Vector3 JoltRope3D::get_pin_position(int p_index) const {
	const Attachment *attachment = attachments.getptr(p_index);
	ERR_FAIL_NULL_V_MSG(attachment, Vector3(), "Rope point is not pinned.");

	return attachment->static_position;
}

void JoltRope3D::attach_point_to_body(int p_index, RID p_body_rid, JoltBody3D *p_body, const Vector3 &p_local_offset) {
	ERR_FAIL_INDEX(p_index, (int)positions.size());

	Attachment attachment;
	_carry_locks(attachment, p_index, ATTACH_BODY, p_body_rid);
	attachment.mode = ATTACH_BODY;
	attachment.body_rid = p_body_rid;
	attachment.body = p_body;
	attachment.local_offset = p_local_offset;
	attachments[p_index] = attachment;

	lra_dirty = true;
}

void JoltRope3D::set_attachment_flag(int p_index, PhysicsServer3D::RopeAttachmentFlag p_flag, bool p_enabled) {
	Attachment *attachment = attachments.getptr(p_index);
	if (attachment == nullptr) {
		return;
	}

	switch (p_flag) {
		case PhysicsServer3D::ROPE_ATTACHMENT_LOCK_TWIST: {
			if (attachment->lock_twist != p_enabled) {
				attachment->lock_twist = p_enabled;
				// Re-capture rather than reuse the old pose: a lock switched off and on again should
				// fasten the rope where it is now, not drag it back to where it was.
				attachment->twist_captured = false;
			}
		} break;
		case PhysicsServer3D::ROPE_ATTACHMENT_LOCK_DIRECTION: {
			if (attachment->lock_direction != p_enabled) {
				attachment->lock_direction = p_enabled;
				attachment->direction_captured = false;
			}
		} break;
		case PhysicsServer3D::ROPE_ATTACHMENT_SOLVER_LIMIT: {
			if (attachment->solver_limit != p_enabled) {
				attachment->solver_limit = p_enabled;
				if (!p_enabled) {
					_release_limit_constraint(*attachment);
				}
			}
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled rope attachment flag: '%d'. This should not happen. Please report this.", p_flag));
		}
	}
}

bool JoltRope3D::get_attachment_flag(int p_index, PhysicsServer3D::RopeAttachmentFlag p_flag) const {
	const Attachment *attachment = attachments.getptr(p_index);
	if (attachment == nullptr) {
		return false;
	}

	switch (p_flag) {
		case PhysicsServer3D::ROPE_ATTACHMENT_LOCK_TWIST:
			return attachment->lock_twist;
		case PhysicsServer3D::ROPE_ATTACHMENT_LOCK_DIRECTION:
			return attachment->lock_direction;
		case PhysicsServer3D::ROPE_ATTACHMENT_SOLVER_LIMIT:
			return attachment->solver_limit;
		default:
			ERR_FAIL_V_MSG(false, vformat("Unhandled rope attachment flag: '%d'. This should not happen. Please report this.", p_flag));
	}
}

void JoltRope3D::set_attachment_param(int p_index, PhysicsServer3D::RopeAttachmentParam p_param, float p_value) {
	Attachment *attachment = attachments.getptr(p_index);
	if (attachment == nullptr) {
		return;
	}

	switch (p_param) {
		case PhysicsServer3D::ROPE_ATTACHMENT_PARAM_DIRECTION_COMPLIANCE: {
			attachment->direction_compliance = MAX(p_value, 0.0f);
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled rope attachment parameter: '%d'. This should not happen. Please report this.", p_param));
		}
	}
}

float JoltRope3D::get_attachment_param(int p_index, PhysicsServer3D::RopeAttachmentParam p_param) const {
	const Attachment *attachment = attachments.getptr(p_index);
	if (attachment == nullptr) {
		return 0.0f;
	}

	switch (p_param) {
		case PhysicsServer3D::ROPE_ATTACHMENT_PARAM_DIRECTION_COMPLIANCE:
			return attachment->direction_compliance;
		default:
			ERR_FAIL_V_MSG(0.0f, vformat("Unhandled rope attachment parameter: '%d'. This should not happen. Please report this.", p_param));
	}
}

// Carries an existing attachment's lock settings, and the pose it captured them in, onto the
// replacement.
//
// Attachments get re-declared whenever the scene reports a change, and for a `RopeAttachment3D`
// parented to a moving body that is every single frame. A lock re-captures the rope's current pose
// the first time it is solved, so without this it would re-capture continuously and never build up
// any twist at all -- the feature would appear to do nothing. The captured pose only carries over
// when the replacement refers to the same thing; anything else genuinely is a new attachment.
void JoltRope3D::_carry_locks(Attachment &p_attachment, int p_index, AttachMode p_mode, const RID &p_body_rid) {
	Attachment *existing = attachments.getptr(p_index);
	if (existing == nullptr) {
		return;
	}

	p_attachment.lock_twist = existing->lock_twist;
	p_attachment.lock_direction = existing->lock_direction;
	p_attachment.direction_compliance = existing->direction_compliance;
	p_attachment.solver_limit = existing->solver_limit;

	if (existing->mode != p_mode || (p_mode == ATTACH_BODY && existing->body_rid != p_body_rid)) {
		// Genuinely a different attachment. Its Jolt constraint describes the old one, and the space
		// is still holding it, so it has to be unregistered rather than just dropped.
		_release_limit_constraint(*existing);
		return;
	}

	p_attachment.locked_normal = existing->locked_normal;
	p_attachment.locked_direction = existing->locked_direction;
	p_attachment.twist_captured = existing->twist_captured;
	p_attachment.direction_captured = existing->direction_captured;
	p_attachment.twist_target = existing->twist_target;
	p_attachment.twist_segment = existing->twist_segment;

	// The Jolt constraint moves across rather than being rebuilt. It is re-declared every frame for an
	// attachment node parented to a moving body, and a constraint recreated that often never keeps the
	// impulse it accumulated last step -- so it solves cold every time and visibly under-holds. It is
	// also registered with the space, so dropping the reference here would leave the space with a
	// constraint nothing owns.
	p_attachment.limit_constraint = existing->limit_constraint;
	p_attachment.limit_constraint_body = existing->limit_constraint_body;
	p_attachment.limit_constraint_anchor_body = existing->limit_constraint_anchor_body;
	p_attachment.limit_constraint_point = existing->limit_constraint_point;
	p_attachment.limit_constraint_offset = existing->limit_constraint_offset;
	p_attachment.limit_constraint_distance = existing->limit_constraint_distance;
	existing->limit_constraint = nullptr;
}

void JoltRope3D::detach_point(int p_index) {
	if (Attachment *attachment = attachments.getptr(p_index)) {
		_release_limit_constraint(*attachment);
	}

	if (attachments.erase(p_index)) {
		lra_dirty = true;
	}
}

void JoltRope3D::remove_all_attachments() {
	if (!attachments.is_empty()) {
		_release_all_limit_constraints();
		attachments.clear();
		lra_dirty = true;
	}
}

void JoltRope3D::detach_from_body(RID p_body_rid) {
	LocalVector<int> stale;

	for (const KeyValue<int, Attachment> &E : attachments) {
		if (E.value.mode == ATTACH_BODY && E.value.body_rid == p_body_rid) {
			stale.push_back(E.key);
		}
	}

	for (int index : stale) {
		// The body is going away, and Jolt requires a constraint to be gone before either of its
		// bodies is.
		if (Attachment *attachment = attachments.getptr(index)) {
			_release_limit_constraint(*attachment);
		}
		attachments.erase(index);
	}

	if (!stale.is_empty()) {
		lra_dirty = true;
	}
}

void JoltRope3D::apply_point_impulse(int p_index, const Vector3 &p_impulse) {
	ERR_FAIL_INDEX(p_index, (int)positions.size());
	velocities[p_index] += p_impulse * inv_masses[p_index];
}

void JoltRope3D::apply_central_impulse(const Vector3 &p_impulse) {
	const uint32_t count = positions.size();
	if (count == 0) {
		return;
	}

	const Vector3 share = p_impulse / (real_t)count;
	for (uint32_t i = 0; i < count; i++) {
		velocities[i] += share * inv_masses[i];
	}
}

void JoltRope3D::_release_limit_constraint(Attachment &p_attachment) {
	if (p_attachment.limit_constraint == nullptr) {
		return;
	}

	if (space != nullptr) {
		space->remove_joint(p_attachment.limit_constraint);
	}

	p_attachment.limit_constraint = nullptr;
	p_attachment.limit_constraint_body = JPH::BodyID();
	p_attachment.limit_constraint_anchor_body = JPH::BodyID();
	p_attachment.limit_constraint_distance = -1.0f;
}

void JoltRope3D::_release_all_limit_constraints() {
	for (KeyValue<int, Attachment> &E : attachments) {
		_release_limit_constraint(E.value);
	}
}

// Hands the limit joint to Jolt as a real `DistanceConstraint`, so a body tied to the rope is bounded
// by it inside the solver rather than afterwards.
//
// The rope's own version of this constraint stays in `_solve_velocities()` and is used whenever a
// constraint could not be built -- a rope with no immovable anchor to measure from, an anchor whose
// body has left the space. The two are never both active on the same attachment, because between
// them they would take the same violation out of the body twice.
void JoltRope3D::_update_limit_constraint(Attachment &p_attachment, const Attachment *p_anchor) {
	const JoltBody3D *body = p_attachment.body;
	JPH::Body *jolt_body = (body != nullptr && body->in_space()) ? body->get_jolt_body() : nullptr;

	if (space == nullptr || !inextensible || !p_attachment.solver_limit || p_anchor == nullptr || jolt_body == nullptr || !jolt_body->IsDynamic()) {
		_release_limit_constraint(p_attachment);
		return;
	}

	// Where the rope is anchored, and what that anchor is attached to. A static pin is a point in the
	// world; an anchor on a static or kinematic body rides along with it, so the constraint is built
	// against that body and follows it for free.
	JPH::Body *anchor_body = &JPH::Body::sFixedToWorld;
	Vector3 anchor_point = p_anchor->target;
	// What identifies this anchor from one frame to the next. A point in the world for a static pin;
	// for an anchor riding a body -- another hand holding the same rope, say -- the world point moves
	// every frame while the constraint it describes does not, so the body-local offset is the thing
	// that has to stay put.
	Vector3 anchor_key = p_anchor->target;

	if (p_anchor->mode == ATTACH_BODY) {
		const JoltBody3D *other = p_anchor->body;
		JPH::Body *other_jolt = (other != nullptr && other->in_space()) ? other->get_jolt_body() : nullptr;
		if (other_jolt == nullptr) {
			_release_limit_constraint(p_attachment);
			return;
		}
		anchor_body = other_jolt;
		anchor_key = p_anchor->local_offset;
	}

	const JPH::BodyID anchor_id = anchor_body->GetID();

	// Rebuilt only when it actually describes something different. Jolt bakes the world-space points
	// into body-local frames when the constraint is created, so a moved anchor or a re-tied knot needs
	// a new one -- but the offsets round-trip through the scene layer unchanged every frame, so in
	// steady state nothing here fires.
	const bool same =
			p_attachment.limit_constraint != nullptr &&
			p_attachment.limit_constraint_body == jolt_body->GetID() &&
			p_attachment.limit_constraint_anchor_body == anchor_id &&
			p_attachment.limit_constraint_point.is_equal_approx(anchor_key) &&
			p_attachment.limit_constraint_offset.is_equal_approx(p_attachment.local_offset);

	if (same) {
		if (!Math::is_equal_approx(p_attachment.limit_constraint_distance, p_attachment.limit_distance)) {
			static_cast<JPH::DistanceConstraint *>(p_attachment.limit_constraint.GetPtr())->SetDistance(0.0f, p_attachment.limit_distance);
			p_attachment.limit_constraint_distance = p_attachment.limit_distance;
		}
		return;
	}

	_release_limit_constraint(p_attachment);

	JPH::DistanceConstraintSettings settings;
	settings.mSpace = JPH::EConstraintSpace::WorldSpace;
	settings.mPoint1 = to_jolt_r(anchor_point);
	settings.mPoint2 = to_jolt_r(p_attachment.target);
	// One-sided, exactly as the rope's own limit is: a rope stops a body being pulled past its length
	// and does nothing whatsoever about it moving back toward the anchor.
	settings.mMinDistance = 0.0f;
	settings.mMaxDistance = p_attachment.limit_distance;

	JPH::TwoBodyConstraint *constraint = static_cast<JPH::TwoBodyConstraint *>(settings.Create(*anchor_body, *jolt_body));
	if (constraint == nullptr) {
		return;
	}

	p_attachment.limit_constraint = constraint;
	p_attachment.limit_constraint_body = jolt_body->GetID();
	p_attachment.limit_constraint_anchor_body = anchor_id;
	p_attachment.limit_constraint_point = anchor_key;
	p_attachment.limit_constraint_offset = p_attachment.local_offset;
	p_attachment.limit_constraint_distance = p_attachment.limit_distance;

	space->add_joint(constraint);
}

void JoltRope3D::_update_attachment_targets() {
	const uint32_t count = positions.size();

	for (uint32_t i = 0; i < count; i++) {
		inv_masses[i] = base_inv_masses[i];
	}

	for (KeyValue<int, Attachment> &E : attachments) {
		Attachment &attachment = E.value;
		const int index = E.key;

		attachment.collider = -1;
		attachment.soft = false;

		if (index < 0 || index >= (int)count) {
			continue;
		}

		switch (attachment.mode) {
			case ATTACH_STATIC: {
				attachment.target = attachment.static_position;
				attachment.holder_basis = Basis();
				inv_masses[index] = 0.0f;
			} break;

			case ATTACH_BODY: {
				const JoltBody3D *body = attachment.body;
				const JPH::Body *jolt_body = (body != nullptr && body->in_space()) ? body->get_jolt_body() : nullptr;

				if (jolt_body == nullptr) {
					// The target has left the space. Leave the particle free rather than snapping
					// it to a stale position.
					attachment.target = positions[index];
					break;
				}

				const Transform3D body_transform(to_godot(jolt_body->GetRotation()), to_godot(jolt_body->GetPosition()));
				attachment.target = body_transform.xform(attachment.local_offset);
				attachment.holder_basis = body_transform.basis;

				// A *dynamic* body is always solved as a real constraint, regardless of
				// `two_way_coupling`. Hard-pinning the rope end to a dynamic body would let the
				// body drag that end wherever it liked and the rope would have to stretch to
				// follow -- unboundedly, since a pinned particle cannot resist. An inextensible
				// rope tied to a crate has to be able to hold the crate. `two_way_coupling`
				// therefore governs only what the rope does to bodies it *collides* with.
				//
				// Static and kinematic bodies keep the hard pin, which is both correct (the rope
				// genuinely cannot move them) and cheaper.
				if (jolt_body->IsDynamic()) {
					// Resolved here rather than inside the substep loop, so the collider array
					// cannot grow (and invalidate references) mid-solve.
					attachment.collider = _resolve_collider(jolt_body->GetID());
					attachment.soft = attachment.collider >= 0;
				}

				if (!attachment.soft) {
					inv_masses[index] = 0.0f;
				}
			} break;

			case ATTACH_NONE:
				break;
		}
	}

	// Second pass: give every soft (dynamic-body) attachment an anchor to be limited against.
	//
	// The bound is the rest length of rope between the two, and it is what stops a heavy body simply
	// out-massing the rope's own particles and dragging the chain apart.
	//
	// Anchors are not only the immovable attachments. A body whose limit Jolt owns is pinned hard and
	// its particle is immovable to the rope (see the sweep below), so it can anchor the span past it
	// -- and it *must*, or nothing constrains that span at all. Limiting everything against the
	// nearest hook instead leaves the rope between two attachments completely free: two hands on one
	// rope could be pulled apart indefinitely because each was only ever measured against the hook,
	// and a load below a grabbed point would be measured on a straight line to the hook while the
	// rope actually has to run out to the grab and back, so the length solve dragged the rope end off
	// the load it was tied to.
	//
	// Grounding therefore spreads outward from the genuinely immovable attachments, always taking the
	// closest unanchored one next. Expanding nearest-first is what keeps every chain terminating at
	// something that cannot move: picking anchors independently would happily point two neighbouring
	// attachments at each other and ground neither.
	if (cumulative_rest.size() == count) {
		for (KeyValue<int, Attachment> &E : attachments) {
			E.value.limit_anchor = -1;
			E.value.limit_distance = 0.0f;
		}

		LocalVector<int> anchors;
		for (const KeyValue<int, Attachment> &E : attachments) {
			if (!E.value.soft && E.value.mode != ATTACH_NONE && E.key >= 0 && E.key < (int)count) {
				anchors.push_back(E.key);
			}
		}

		while (!anchors.is_empty()) {
			int best_key = -1;
			int best_anchor = -1;
			float best_distance = 0.0f;

			for (const KeyValue<int, Attachment> &E : attachments) {
				const Attachment &attachment = E.value;
				if (!attachment.soft || attachment.limit_anchor >= 0 || E.key < 0 || E.key >= (int)count) {
					continue;
				}

				for (int anchor : anchors) {
					if (anchor == E.key) {
						continue;
					}

					const float distance = Math::abs(cumulative_rest[E.key] - cumulative_rest[anchor]);
					if (best_key < 0 || distance < best_distance) {
						best_key = E.key;
						best_anchor = anchor;
						best_distance = distance;
					}
				}
			}

			if (best_key < 0) {
				break;
			}

			Attachment *attachment = attachments.getptr(best_key);
			attachment->limit_anchor = best_anchor;
			attachment->limit_distance = best_distance;

			// Only an attachment that will actually be pinned hard becomes an anchor for the next
			// layer. One the rope merely pulls on is still free to move, so it can bound nothing.
			if (attachment->solver_limit) {
				anchors.push_back(best_key);
			} else {
				// Nothing further can ground through it, and it is anchored now, so the search shrinks
				// on the next round either way.
			}
		}

		// Second sweep, because the anchor an attachment settles on may be an entry the loop above had
		// not reached yet, and looking it up needs the map to be final.
		for (KeyValue<int, Attachment> &E : attachments) {
			Attachment &attachment = E.value;
			const Attachment *anchor = (attachment.soft && attachment.limit_anchor >= 0)
					? attachments.getptr(attachment.limit_anchor)
					: nullptr;
			_update_limit_constraint(attachment, anchor);

			// An attachment whose limit Jolt owns is pinned hard, so its particle has to be immovable
			// like any other hard pin. Leaving it with a mass meant every other constraint -- contacts
			// especially -- was free to shove it somewhere else, and the pin teleported it back on the
			// next substep. Four rounds of that per frame, and the particles either side were left
			// visibly shaking.
			if (attachment.limit_constraint != nullptr && E.key >= 0 && E.key < (int)count) {
				inv_masses[E.key] = 0.0f;
			}
		}
	} else {
		_release_all_limit_constraints();
	}
}

void JoltRope3D::_update_lra() {
	const uint32_t count = positions.size();

	lra_anchor.resize(count);
	lra_distance.resize(count);

	for (uint32_t i = 0; i < count; i++) {
		lra_anchor[i] = -1;
		lra_distance[i] = 0.0f;
	}

	lra_dirty = false;

	if (count == 0) {
		return;
	}

	if (cumulative_rest.size() != count) {
		return;
	}
	const LocalVector<float> &cumulative = cumulative_rest;

	for (const KeyValue<int, Attachment> &E : attachments) {
		const int anchor = E.key;
		if (anchor < 0 || anchor >= (int)count || E.value.mode == ATTACH_NONE) {
			continue;
		}

		for (uint32_t i = 0; i < count; i++) {
			const float distance = Math::abs(cumulative[i] - cumulative[anchor]);
			if (lra_anchor[i] < 0 || distance < lra_distance[i]) {
				lra_anchor[i] = anchor;
				lra_distance[i] = distance;
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
/* Collision gathering                                                        */
/* -------------------------------------------------------------------------- */

int JoltRope3D::_resolve_collider(const JPH::BodyID &p_body_id) {
	const uint32_t key = p_body_id.GetIndexAndSequenceNumber();

	if (const int *existing = collider_lookup.getptr(key)) {
		return *existing;
	}

	const JPH::Body *jolt_body = space->try_get_jolt_body(p_body_id);
	if (jolt_body == nullptr) {
		return -1;
	}

	ColliderInfo info;
	info.body_id = p_body_id;
	info.transform = Transform3D(to_godot(jolt_body->GetRotation()), to_godot(jolt_body->GetPosition()));
	info.inv_transform = info.transform.affine_inverse();
	info.com = to_godot(jolt_body->GetCenterOfMassPosition());
	info.linear_velocity = to_godot(jolt_body->GetLinearVelocity());
	info.angular_velocity = to_godot(jolt_body->GetAngularVelocity());
	info.friction = jolt_body->GetFriction();
	info.restitution = jolt_body->GetRestitution();
	info.dynamic = jolt_body->IsDynamic();

	if (info.dynamic) {
		const JPH::MotionProperties *motion = jolt_body->GetMotionProperties();
		info.inv_mass = motion->GetInverseMass();
		info.inv_inertia = to_godot(motion->GetInverseInertiaForRotation(JPH::Mat44::sRotation(jolt_body->GetRotation()))).basis;
	}

	const int index = (int)colliders.size();
	colliders.push_back(info);
	collider_lookup[key] = index;

	return index;
}

void JoltRope3D::_gather_contacts(float p_step) {
	contacts.clear();

	const uint32_t count = positions.size();
	if (count < 2) {
		return;
	}

	const JoltRopeQueryFilter3D filter(*space, collision_mask, exceptions);

	// One broadphase pass over the whole rope first. A hanging or swinging rope touches nothing on
	// most frames, and this turns that common case into a single AABB test instead of one
	// narrow-phase query per segment.
	{
		// Grown by how far the fastest particle can travel this frame, so a rope swinging into
		// something is not rejected here before its segments ever reach the narrow phase.
		float max_speed_sq = 0.0f;
		for (uint32_t i = 0; i < count; i++) {
			max_speed_sq = MAX(max_speed_sq, (float)velocities[i].length_squared());
		}

		AABB query_bounds = bounds;
		query_bounds.grow_by(ROPE_SPECULATIVE_MARGIN + Math::sqrt(max_speed_sq) * p_step);

		JoltQueryCollectorAny<JPH::CollideShapeBodyCollector> collector;
		space->get_broad_phase_query().CollideAABox(to_jolt(query_bounds), collector, filter, filter);

		if (!collector.had_hit()) {
			return;
		}
	}

	JPH::CollideShapeSettings settings;
	settings.mMaxSeparationDistance = ROPE_SPECULATIVE_MARGIN;
	settings.mBackFaceMode = JPH::EBackFaceMode::IgnoreBackFaces;
	// Without this a rope sliding across a triangle mesh snags on the internal edges between
	// triangles, which is a very visible artifact.
	settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;

	JoltQueryCollectorAll<JPH::CollideShapeCollector, 32> collector;

	for (uint32_t segment = 0; segment + 1 < count; segment++) {
		const Vector3 &from = positions[segment];
		const Vector3 &to = positions[segment + 1];

		const Vector3 delta = to - from;
		const float length = (float)delta.length();

		// `JPH::CapsuleShape` requires a strictly positive cylinder half-height, so a collapsed
		// segment is skipped rather than clamped.
		const float half_height = length * 0.5f;
		if (half_height < ROPE_EPSILON) {
			continue;
		}

		const Vector3 axis = delta / length;
		const Vector3 mid = (from + to) * 0.5f;

		const JPH::CapsuleShape capsule(half_height, radius);
		const JPH::RMat44 transform = to_jolt_r(Transform3D(capsule_basis_for_axis(axis), mid));

		collector.reset();
		space->get_narrow_phase_query().CollideShape(
				&capsule, JPH::Vec3::sOne(), transform, settings, JPH::RVec3::sZero(),
				collector, filter, filter, filter);

		const int hit_count = collector.get_hit_count();

		int kept = 0;
		for (int i = 0; i < hit_count && kept < ROPE_MAX_CONTACTS_PER_SEGMENT; i++) {
			const JPH::CollideShapeResult &hit = collector.get_hit(i);

			if (hit.mPenetrationAxis.LengthSq() < ROPE_EPSILON) {
				continue;
			}

			const int collider_index = _resolve_collider(hit.mBodyID2);
			if (collider_index < 0) {
				continue;
			}

			const ColliderInfo &info = colliders[collider_index];

			// `mPenetrationAxis` is the direction that moves shape 2 out of shape 1, so negating it
			// gives the direction that pushes the rope away from the collider.
			const Vector3 normal = -to_godot(hit.mPenetrationAxis.Normalized());
			const Vector3 point = to_godot(hit.mContactPointOn2);

			Contact contact;
			contact.segment = (int)segment;
			contact.collider = collider_index;
			contact.local_point = info.inv_transform.xform(point);
			contact.local_normal = info.inv_transform.basis.xform(normal);

			// Where along the segment the contact sits, so the correction is split between the two
			// endpoints instead of being dumped entirely on one of them.
			contact.t = CLAMP((float)(point - from).dot(axis) / length, 0.0f, 1.0f);

			contacts.push_back(contact);
			kept++;
		}
	}
}

/* -------------------------------------------------------------------------- */
/* Solver                                                                     */
/* -------------------------------------------------------------------------- */

// `JoltSpace3D` zeroes Jolt's own system gravity (`jolt_space_3d.cpp:147`) because Godot applies
// gravity through Areas instead, per body. Ropes have no Area membership tracking of their own yet,
// so they take the space's default area — which is what governs a rope in all but the case of an
// `Area3D` gravity override. `compute_gravity()` is sampled per particle so point gravity bends the
// rope correctly rather than pulling it as a rigid whole.
void JoltRope3D::_update_gravity() {
	const uint32_t count = positions.size();
	gravity_cache.resize(count);

	const JoltArea3D *default_area = space->get_default_area();
	if (default_area == nullptr) {
		for (uint32_t i = 0; i < count; i++) {
			gravity_cache[i] = Vector3();
		}
		return;
	}

	for (uint32_t i = 0; i < count; i++) {
		gravity_cache[i] = default_area->compute_gravity(positions[i]) * gravity_scale;
	}
}

// Damps deformation without damping motion.
//
// The obvious form -- scale every particle's velocity toward zero -- damps a rope that is merely
// *moving* exactly as hard as one that is flexing, so a rope swinging as a perfectly straight line
// bleeds energy even though nothing about it is deforming. What it should damp is the difference
// between what the rope is doing and the rigid motion that best describes it, which is the classic
// treatment from Mueller et al., *Position Based Dynamics*, section 3.5: recover the rope's overall
// translation and rotation from its own momentum, then damp only the residual.
//
// A settled rope still stops. A swinging one keeps swinging.
void JoltRope3D::_solve_damping(float p_step) {
	const uint32_t count = positions.size();
	const real_t strength = CLAMP((real_t)linear_damping * (real_t)p_step, (real_t)0.0, (real_t)1.0);
	if (count < 2 || strength <= (real_t)0.0) {
		return;
	}

	Vector3 centre;
	Vector3 momentum;
	real_t total_mass_sum = 0.0;

	for (uint32_t i = 0; i < count; i++) {
		if (inv_masses[i] <= 0.0f) {
			// A pinned particle has infinite mass, so it would swamp the fit and, being immovable,
			// there is nothing to damp about it either.
			continue;
		}
		const real_t mass = (real_t)1.0 / (real_t)inv_masses[i];
		centre += positions[i] * mass;
		momentum += velocities[i] * mass;
		total_mass_sum += mass;
	}

	if (total_mass_sum <= (real_t)ROPE_EPSILON) {
		return;
	}

	centre /= total_mass_sum;
	const Vector3 linear = momentum / total_mass_sum;

	Vector3 angular_momentum;
	Basis inertia;

	for (uint32_t i = 0; i < count; i++) {
		if (inv_masses[i] <= 0.0f) {
			continue;
		}
		const real_t mass = (real_t)1.0 / (real_t)inv_masses[i];
		const Vector3 offset = positions[i] - centre;

		angular_momentum += offset.cross(velocities[i] * mass);

		// The point-mass inertia tensor, m * (|r|^2 * I - r r^T), accumulated directly.
		const real_t r2 = offset.length_squared();
		for (int a = 0; a < 3; a++) {
			for (int b = 0; b < 3; b++) {
				inertia[a][b] += mass * ((a == b ? r2 : (real_t)0.0) - offset[a] * offset[b]);
			}
		}
	}

	// A perfectly straight rope of point masses has no moment of inertia about its own axis at all,
	// which makes the tensor singular. Nudging the diagonal by a fraction of its own trace leaves the
	// two well-conditioned axes alone and turns the degenerate one into "no rotation about it",
	// which is exactly right -- a line of points cannot spin about the line.
	const real_t trace = inertia[0][0] + inertia[1][1] + inertia[2][2];
	const real_t regulariser = MAX(trace, (real_t)ROPE_EPSILON) * (real_t)1e-4;
	for (int a = 0; a < 3; a++) {
		inertia[a][a] += regulariser;
	}

	const Vector3 angular = inertia.inverse().xform(angular_momentum);

	for (uint32_t i = 0; i < count; i++) {
		if (inv_masses[i] <= 0.0f) {
			continue;
		}
		const Vector3 rigid = linear + angular.cross(positions[i] - centre);
		velocities[i] += (rigid - velocities[i]) * strength;
	}
}

void JoltRope3D::_integrate(float p_step) {
	const uint32_t count = positions.size();

	for (uint32_t i = 0; i < count; i++) {
		prev_positions[i] = positions[i];

		if (inv_masses[i] <= 0.0f) {
			continue;
		}

		velocities[i] += gravity_cache[i] * p_step;

		if (drag > 0.0f) {
			const Vector3 velocity = velocities[i];
			const real_t speed = velocity.length();

			if (speed > (real_t)ROPE_EPSILON) {
				// Aerodynamic drag: quadratic in speed, as real drag is, and split against the rope's
				// own direction because a cable's resistance is dominated by the component across its
				// axis. The previous form reduced to `v *= 1 - drag * h`, which is linear and was
				// simply a second copy of `linear_damping` under a different name.
				Vector3 tangent;
				if (i + 1 < count) {
					tangent += positions[i + 1] - positions[i];
				}
				if (i > 0) {
					tangent += positions[i] - positions[i - 1];
				}

				Vector3 deceleration;
				if (tangent.length_squared() > (real_t)ROPE_EPSILON) {
					tangent.normalize();
					const Vector3 along = tangent * velocity.dot(tangent);
					const Vector3 across = velocity - along;
					deceleration = across * (across.length() * (real_t)drag) +
							along * (along.length() * (real_t)drag * (real_t)ROPE_TANGENTIAL_DRAG_RATIO);
				} else {
					deceleration = velocity * (speed * (real_t)drag);
				}

				// Clamped so an explicit integration of a quadratic law can never reverse the very
				// velocity it is opposing.
				const real_t change = deceleration.length() * (real_t)p_step;
				if (change > speed) {
					deceleration *= speed / change;
				}

				velocities[i] -= deceleration * p_step;
			}
		}

		positions[i] += velocities[i] * p_step;
	}
}

void JoltRope3D::_solve_distance(float p_step) {
	const uint32_t segment_count = rest_lengths.size();
	if (segment_count == 0) {
		return;
	}

	// XPBD compliance is scaled by the substep duration, which is what makes stiffness independent
	// of both the timestep and the substep count.
	const float alpha = stretch_compliance / (p_step * p_step);

	for (uint32_t i = 0; i < segment_count; i++) {
		lambdas[i] = 0.0f;
	}

	// Two sweeps in opposite directions. A single-direction Gauss-Seidel sweep biases the solution
	// toward whichever end it starts from, which shows up as a rope that sags at the far end.
	for (int pass = 0; pass < 2; pass++) {
		const bool forward = (pass == 0);

		for (uint32_t s = 0; s < segment_count; s++) {
			const uint32_t i = forward ? s : (segment_count - 1 - s);

			const float w0 = inv_masses[i];
			const float w1 = inv_masses[i + 1];
			const float w = w0 + w1;
			if (w <= 0.0f) {
				continue;
			}

			const Vector3 delta = positions[i + 1] - positions[i];
			const float length = (float)delta.length();
			if (length < ROPE_EPSILON) {
				continue;
			}

			const Vector3 normal = delta / length;
			const float error = length - rest_lengths[i];

			const float delta_lambda = (-error - alpha * lambdas[i]) / (w + alpha);
			lambdas[i] += delta_lambda;

			const Vector3 correction = normal * delta_lambda;
			positions[i] -= correction * w0;
			positions[i + 1] += correction * w1;
		}
	}
}

void JoltRope3D::_solve_bending(float p_step) {
	const uint32_t bend_count = bend_lambdas.size();
	if (bend_count == 0 || bend_compliance >= ROPE_BEND_DISABLED_COMPLIANCE) {
		return;
	}

	const float alpha = bend_compliance / (p_step * p_step);

	for (uint32_t i = 0; i < bend_count; i++) {
		bend_lambdas[i] = 0.0f;
	}

	// Symmetric, and with proper XPBD lambda accumulation, for the same reasons `_solve_distance()`
	// is. It matters far more here: bending is what makes a rope hold a shape along its whole
	// length, so the stiffness has to travel from one end to the other, and Gauss-Seidel carries it
	// about one particle per sweep. A single forward sweep per substep therefore reached only four
	// particles of a twenty-one particle rope at the default substep count -- which made the
	// compliance slider look inert across its entire range, because what limited the result was
	// convergence rather than the stiffness being asked for.
	for (int pass = 0; pass < ROPE_BEND_SWEEPS; pass++) {
		const bool forward = (pass % 2) == 0;

		for (uint32_t s = 0; s < bend_count; s++) {
			const uint32_t i = forward ? s : (bend_count - 1 - s);

			const float w0 = inv_masses[i];
			const float w1 = inv_masses[i + 1];
			const float w2 = inv_masses[i + 2];

			// Gradient magnitudes are 1 for the middle particle and 1/2 for each neighbour, so the
			// neighbours contribute a quarter of their inverse mass to the effective one.
			const float w = w1 + 0.25f * (w0 + w2);
			if (w <= 0.0f) {
				continue;
			}

			// How far the middle particle stands off the line joining its neighbours. Zero when the
			// three are collinear, which is the rest state, and -- unlike the chord between the
			// outer two -- it grows linearly with the fold angle rather than quadratically.
			const Vector3 offset = positions[i + 1] - (positions[i] + positions[i + 2]) * 0.5f;
			const float error = (float)offset.length();
			if (error < ROPE_EPSILON) {
				continue;
			}

			const Vector3 normal = offset / error;

			const float delta_lambda = (-error - alpha * bend_lambdas[i]) / (w + alpha);
			bend_lambdas[i] += delta_lambda;

			const Vector3 correction = normal * delta_lambda;

			// The middle particle moves toward the chord and the neighbours move the other way, each
			// by half as much -- so the triple straightens without the group drifting.
			positions[i + 1] += correction * w1;
			positions[i] -= correction * (0.5f * w0);
			positions[i + 2] -= correction * (0.5f * w2);
		}
	}
}

// Clamps every segment to at most its rest length, symmetrically and in both sweep directions.
// One-sided, so it never compresses the rope -- it only removes stretch. A per-segment upper
// bound is also a total-length upper bound, which is what makes `inextensible` mean something for
// a rope that has no anchor for `_solve_lra()` to measure from.
void JoltRope3D::_solve_strain_limit() {
	if (!inextensible) {
		return;
	}

	const uint32_t segment_count = rest_lengths.size();

	for (int pass = 0; pass < 2; pass++) {
		const bool forward = (pass == 0);

		for (uint32_t s = 0; s < segment_count; s++) {
			const uint32_t i = forward ? s : (segment_count - 1 - s);

			const float w0 = inv_masses[i];
			const float w1 = inv_masses[i + 1];
			const float w = w0 + w1;
			if (w <= 0.0f) {
				continue;
			}

			const Vector3 delta = positions[i + 1] - positions[i];
			const float length = (float)delta.length();
			const float limit = rest_lengths[i];

			if (length <= limit || length < ROPE_EPSILON) {
				continue;
			}

			// Note that `prev_positions` is deliberately NOT moved along with this. It is tempting:
			// removing stretch looks like correcting a solver artifact rather than releasing stored
			// energy. But `_update_velocities()` derives velocity from `(pos - prev)`, so cancelling
			// this pass there would leave the particle holding exactly the velocity that violated
			// the constraint -- it would stretch again next substep, be clamped again, and the
			// violating velocity would compound instead of being removed.
			const Vector3 correction = delta * ((length - limit) / (length * w));
			positions[i] += correction * w0;
			positions[i + 1] -= correction * w1;
		}
	}
}

void JoltRope3D::_solve_lra() {
	if (!inextensible) {
		return;
	}

	const uint32_t count = positions.size();
	if (lra_anchor.size() != count) {
		return;
	}

	for (uint32_t i = 0; i < count; i++) {
		const int anchor = lra_anchor[i];
		if (anchor < 0 || (uint32_t)anchor == i || inv_masses[i] <= 0.0f) {
			continue;
		}

		const Vector3 delta = positions[i] - positions[anchor];
		const float length = (float)delta.length();
		const float limit = lra_distance[i];

		if (length <= limit || length < ROPE_EPSILON) {
			continue;
		}

		// The anchor is treated as immovable here, deliberately. The whole point of this constraint
		// is to enforce the global length limit in a single pass; letting it push the anchor around
		// would reintroduce exactly the slow propagation it exists to avoid.
		positions[i] = positions[anchor] + delta * (limit / length);
	}
}

void JoltRope3D::_solve_attachments(float p_step) {
	for (KeyValue<int, Attachment> &E : attachments) {
		Attachment &attachment = E.value;
		const int index = E.key;

		if (index < 0 || index >= (int)positions.size() || attachment.mode == ATTACH_NONE) {
			continue;
		}

		if (!attachment.soft || attachment.limit_constraint != nullptr) {
			// Hard pin: static, kinematic, or a dynamic body whose limit Jolt now owns.
			//
			// Once there is a real constraint on the body, the rope has nothing left to tell it. The
			// soft path below would still push on it every substep out of the residual gap between the
			// knot and the rope's last particle, and that push is derived from a lagging measurement,
			// so it is dissipative -- it cost a swinging load 15 % of its energy over fifteen seconds
			// against 0.75 % without it. Splitting the two responsibilities is what the constraint was
			// for: the body owns where the rope end is, and the constraint owns how far the body may
			// go.
			positions[index] = attachment.target;
			_solve_direction_lock(attachment, index, p_step);
			continue;
		}

		// Soft attachment to a dynamic body, solved with the body's generalized inverse mass so a
		// heavy body barely moves while a light one gets dragged. This is what makes towing and
		// load-bearing behave rather than just teleporting the rope end.
		ColliderInfo &info = colliders[attachment.collider];

		// Against the proxy, not the frame-start pose, so the constraint converges over substeps.
		const Vector3 anchor = info.displaced(attachment.target);

		const Vector3 delta = positions[index] - anchor;
		const float length = (float)delta.length();
		if (length < ROPE_EPSILON) {
			continue;
		}

		const Vector3 normal = delta / length;
		const Vector3 lever = anchor - (info.com + info.position_delta);
		const Vector3 lever_cross = lever.cross(normal);

		const float w_particle = inv_masses[index];
		const float w_body = info.inv_mass + (float)lever_cross.dot(info.inv_inertia.xform(lever_cross));
		const float w = w_particle + w_body;
		if (w <= 0.0f) {
			continue;
		}

		const Vector3 correction = normal * (-length / w);

		positions[index] += correction * w_particle;

		// The body takes the other share of the correction. Recording it on the proxy is what stops
		// the next substep from asking for the whole thing again; recording the impulse separately is
		// what gets handed back to Jolt at the end of the frame.
		info.position_delta -= correction * info.inv_mass;
		info.rotation_delta -= info.inv_inertia.xform(lever.cross(correction));

		info.linear_impulse -= correction / frame_step;
		info.angular_impulse -= lever.cross(correction) / frame_step;

		_solve_direction_lock(attachment, index, p_step);

		// The limit-joint constraint. Solved against the body alone, because the anchor it is
		// measured from is immovable by construction. Without it the rope can only resist through
		// its own particles, whose combined mass is usually far less than the body's -- so a heavy
		// body would keep falling, the rope would be pulled straight past its rest length, and the
		// long-range pass would tear it open in the middle.
		// Skipped when Jolt owns this limit: between them the two would take the same violation out of
		// the body twice.
		if (attachment.limit_anchor < 0 || !inextensible || attachment.limit_constraint != nullptr) {
			continue;
		}

		const Vector3 origin = positions[attachment.limit_anchor];
		const Vector3 span = info.displaced(attachment.target) - origin;
		const float span_length = (float)span.length();
		if (span_length <= attachment.limit_distance || span_length < ROPE_EPSILON) {
			continue;
		}

		const Vector3 span_normal = span / span_length;
		const Vector3 span_lever = info.displaced(attachment.target) - (info.com + info.position_delta);
		const Vector3 span_lever_cross = span_lever.cross(span_normal);

		const float span_w = info.inv_mass + (float)span_lever_cross.dot(info.inv_inertia.xform(span_lever_cross));
		if (span_w <= 0.0f) {
			continue;
		}

		// The body is the only thing here that can move, but a rigid body answers a positional
		// constraint with translation *and* rotation, and `span_w` is precisely the split between
		// them. So the excess is turned into a generalized impulse first and each channel then takes
		// its own share -- applying the whole excess as translation *and* the rotation on top
		// over-corrects by `span_w / inv_mass`. That factor is 1 when the lever from the centre of
		// mass to the attachment happens to lie along the rope, which is exactly the case for a load
		// hanging straight below its knot, and grows as the rope swings away from it. It is why the
		// old form read as a rope that bounced while swinging and behaved perfectly at rest.
		const Vector3 span_impulse = span_normal * (-(span_length - attachment.limit_distance) / span_w);

		const Vector3 anchor_before = origin + span;

		// Proxy only. Unlike the pin above, this constraint contributes no positional impulse of its
		// own: `_solve_velocities()` solves the same limit at velocity level with the position error
		// folded in as a bias, which is the only way to apply both without correcting the same
		// violation twice and flinging the body back past the constraint.
		info.position_delta += span_impulse * info.inv_mass;
		info.rotation_delta += info.inv_inertia.xform(span_lever.cross(span_impulse));

		// Carry the particle along by whatever the attachment point on the body actually moved,
		// rather than by the correction, so the pin stays exact instead of merely close.
		positions[index] += info.displaced(attachment.target) - anchor_before;
	}
}

// Holds the direction the rope leaves an attachment in, so the knot behaves like a cable gland or a
// splice rather than a hook.
//
// This is a *bending* boundary condition, not a twist one, and it works with twist switched off. The
// mechanism is the standard trick for clamping the end of a rod in a position-based solver: invent a
// ghost particle just beyond the attachment, rigidly carried by whatever holds the rope, and run the
// ordinary bending constraint through the triple it forms with the first two real particles. Three
// collinear points means the rope leaves along the captured direction, so no new constraint is
// needed -- only a new place to apply the one already here.
void JoltRope3D::_solve_direction_lock(Attachment &p_attachment, int p_index, float p_step) {
	if (!p_attachment.lock_direction) {
		return;
	}

	const uint32_t count = positions.size();
	const int neighbour = (p_index == 0) ? 1 : p_index - 1;
	if (neighbour < 0 || neighbour >= (int)count) {
		return;
	}

	ColliderInfo *info = (p_attachment.soft && p_attachment.collider >= 0) ? &colliders[p_attachment.collider] : nullptr;

	const Vector3 anchor = (info != nullptr) ? info->displaced(p_attachment.target) : p_attachment.target;

	if (!p_attachment.direction_captured) {
		// Fasten it in the direction the rope already runs, so switching the lock on does not snap
		// the rope to some canonical exit angle.
		Vector3 exit = positions[neighbour] - anchor;
		if (exit.length_squared() < (real_t)ROPE_EPSILON) {
			return;
		}
		p_attachment.locked_direction = p_attachment.holder_basis.transposed().xform(exit.normalized());
		p_attachment.direction_captured = true;
	}

	Vector3 exit = p_attachment.holder_basis.xform(p_attachment.locked_direction);
	if (info != nullptr) {
		// Carry the proxy's rotation, so a body that has already turned this substep is seen to have
		// taken its exit direction with it.
		exit += info->rotation_delta.cross(exit);
	}
	if (exit.length_squared() < (real_t)ROPE_EPSILON) {
		return;
	}
	exit.normalize();

	// The ghost sits one segment back along the exit direction, which puts the three points in a
	// straight line exactly when the rope is leaving the way it was fastened.
	const uint32_t segment = (p_index == 0) ? 0 : (uint32_t)(p_index - 1);
	const float spacing = (segment < rest_lengths.size()) ? rest_lengths[segment] : 1.0f;
	const Vector3 ghost = anchor - exit * spacing;

	const Vector3 offset = positions[p_index] - (ghost + positions[neighbour]) * 0.5f;
	const float error = (float)offset.length();
	if (error < ROPE_EPSILON) {
		return;
	}
	const Vector3 normal = offset / error;

	const float w_index = inv_masses[p_index];
	const float w_neighbour = inv_masses[neighbour];

	// The ghost is not free: it is welded to the holder, so its share of the constraint costs
	// whatever moving that holder costs. For a static pin that is nothing, which is what makes a
	// rope tied to a wall clamp rigidly.
	Vector3 ghost_lever;
	float w_ghost = 0.0f;
	if (info != nullptr) {
		ghost_lever = ghost - (info->com + info->position_delta);
		const Vector3 lever_cross = ghost_lever.cross(normal);
		w_ghost = info->inv_mass + (float)lever_cross.dot(info->inv_inertia.xform(lever_cross));
	}

	// Gradient magnitudes are 1 for the middle point and 1/2 for each outer one, so the outer two
	// contribute a quarter of their inverse mass.
	const float w = w_index + 0.25f * (w_neighbour + w_ghost);
	if (w <= 0.0f) {
		return;
	}

	const float alpha = p_attachment.direction_compliance / (p_step * p_step);
	const float delta_lambda = -error / (w + alpha);
	const Vector3 correction = normal * delta_lambda;

	positions[p_index] += correction * w_index;
	positions[neighbour] -= correction * (0.5f * w_neighbour);

	if (info != nullptr) {
		const Vector3 ghost_impulse = correction * -0.5f;

		info->position_delta += ghost_impulse * info->inv_mass;
		info->rotation_delta += info->inv_inertia.xform(ghost_lever.cross(ghost_impulse));

		info->linear_impulse += ghost_impulse / frame_step;
		info->angular_impulse += ghost_lever.cross(ghost_impulse) / frame_step;
	}
}

void JoltRope3D::_solve_collisions(float p_step) {
	for (const Contact &contact : contacts) {
		ColliderInfo &info = colliders[contact.collider];

		// Re-derive the plane from the collider's transform rather than reusing the world-space
		// plane captured at detection time. Both are cheap, but only this one stays correct when
		// the rope is resting on something that moves.
		const bool couple = two_way_coupling && info.dynamic;

		// Carry the proxy's accumulated motion, so that a body the rope is pressing on is seen to
		// have already given way by the time the next substep looks at it.
		Vector3 plane_point = info.transform.xform(contact.local_point);
		Vector3 plane_normal = info.transform.basis.xform(contact.local_normal);
		if (couple) {
			plane_normal += info.rotation_delta.cross(plane_normal);
			plane_point = info.displaced(plane_point);
		}

		const float normal_length = (float)plane_normal.length();
		if (normal_length < ROPE_EPSILON) {
			continue;
		}
		plane_normal /= normal_length;

		const int i = contact.segment;
		const int j = contact.segment + 1;

		const float a = 1.0f - contact.t;
		const float b = contact.t;

		const Vector3 point = positions[i] * a + positions[j] * b;

		const float separation = (float)(point - plane_point).dot(plane_normal) - radius;
		if (separation >= 0.0f) {
			continue;
		}

		const float w0 = inv_masses[i];
		const float w1 = inv_masses[j];

		// Effective inverse mass of the interpolated contact point, not of either endpoint.
		float w = a * a * w0 + b * b * w1;

		Vector3 lever;
		if (couple) {
			lever = plane_point - (info.com + info.position_delta);
			const Vector3 lever_cross = lever.cross(plane_normal);
			w += info.inv_mass + (float)lever_cross.dot(info.inv_inertia.xform(lever_cross));
		}

		if (w <= 0.0f) {
			continue;
		}

		const Vector3 prev_point = prev_positions[i] * a + prev_positions[j] * b;
		const Vector3 surface_velocity = info.linear_velocity + info.angular_velocity.cross(plane_point - info.com);
		const Vector3 relative_motion = (point - prev_point) - surface_velocity * p_step;

		float depth = -separation;

		// Restitution, applied as extra positional push proportional to the approach speed. A rope
		// barely bounces, so this stays a small correction rather than a full velocity reflection.
		const float combined_restitution = restitution * info.restitution;
		if (combined_restitution > 0.0f) {
			const float approach = (float)relative_motion.dot(plane_normal);
			if (approach < 0.0f) {
				depth -= approach * combined_restitution;
			}
		}

		const Vector3 impulse = plane_normal * (depth / w);

		positions[i] += impulse * (a * w0);
		positions[j] += impulse * (b * w1);

		if (couple) {
			// The body absorbs its share of the push, recorded on the proxy so the next substep
			// sees a surface that has already yielded.
			info.position_delta -= impulse * info.inv_mass;
			info.rotation_delta -= info.inv_inertia.xform(lever.cross(impulse));

			info.linear_impulse -= impulse / frame_step;
			info.angular_impulse -= lever.cross(impulse) / frame_step;
		}

		// Position-level friction, per Macklin et al. "Unified Particle Physics". Damping the
		// tangential motion accumulated since the substep began is stable at any stiffness, unlike
		// a velocity-level Coulomb clamp applied after the position solve. Subtracting the
		// collider's own motion is what carries a rope along on a moving platform instead of
		// dragging it across the surface.
		const float combined_friction = friction * info.friction;
		if (combined_friction > 0.0f) {
			Vector3 tangential = relative_motion;
			tangential -= plane_normal * tangential.dot(plane_normal);

			const Vector3 friction_correction = -tangential * MIN(combined_friction, 1.0f);
			const float tangential_w = a * a * w0 + b * b * w1;
			if (tangential_w > ROPE_EPSILON) {
				positions[i] += friction_correction * (a * w0 / tangential_w);
				positions[j] += friction_correction * (b * w1 / tangential_w);
			}
		}
	}
}

void JoltRope3D::_update_velocities(float p_step) {
	const uint32_t count = positions.size();
	const float inv_step = 1.0f / p_step;

	for (uint32_t i = 0; i < count; i++) {
		velocities[i] = (positions[i] - prev_positions[i]) * inv_step;
	}
}

// Rotates `p_normal` by the smallest rotation carrying `p_from` to `p_to`. Both tangents are unit
// length, so this is Rodrigues with the angle recovered from the cross and dot products.
static Vector3 rope_transport(const Vector3 &p_normal, const Vector3 &p_from, const Vector3 &p_to) {
	const Vector3 axis = p_from.cross(p_to);
	const real_t sine = axis.length();
	const real_t cosine = p_from.dot(p_to);

	if (sine < (real_t)ROPE_EPSILON) {
		// Parallel, or antiparallel and therefore ambiguous -- but a rope segment cannot reverse
		// inside one frame, so treating it as unchanged is right in both cases.
		return p_normal;
	}

	const Vector3 unit_axis = axis / sine;
	const real_t angle = Math::atan2(sine, cosine);
	const real_t c = Math::cos(angle);
	const real_t s = Math::sin(angle);

	return p_normal * c + unit_axis.cross(p_normal) * s + unit_axis * (unit_axis.dot(p_normal) * (1.0 - c));
}

// Rotates `p_normal` about the unit axis `p_axis` by `p_angle`. The two are perpendicular here, so
// the axis-aligned term of Rodrigues drops out.
static Vector3 rope_roll(const Vector3 &p_normal, const Vector3 &p_axis, real_t p_angle) {
	const real_t c = Math::cos(p_angle);
	const real_t s = Math::sin(p_angle);

	return p_normal * c + p_axis.cross(p_normal) * s;
}

// Re-perpendicularises `p_normal` against `p_tangent` and normalises it, falling back to any
// perpendicular if the two have collapsed onto each other.
static Vector3 rope_orthonormalize(const Vector3 &p_normal, const Vector3 &p_tangent) {
	Vector3 normal = p_normal - p_tangent * p_normal.dot(p_tangent);

	if (normal.length_squared() < (real_t)ROPE_EPSILON) {
		const Vector3 reference = Math::abs(p_tangent.y) > (real_t)0.9 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
		normal = reference.cross(p_tangent);
	}

	return normal.normalized();
}

void JoltRope3D::_update_frames() {
	const uint32_t count = positions.size();

	if (count < 2) {
		ref_normals.clear();
		ref_tangents.clear();
		ref_frame_valid = false;
		return;
	}

	const uint32_t segments = count - 1;

	if (ref_normals.size() != segments) {
		ref_normals.resize(segments);
		ref_tangents.resize(segments);
		ref_frame_valid = false;
	}

	Vector3 tangent = positions[1] - positions[0];
	if (tangent.length_squared() < (real_t)ROPE_EPSILON) {
		tangent = Vector3(0, 1, 0);
	}
	tangent.normalize();

	if (ref_frame_valid) {
		// Segment 0 is the only frame carried in time, and it is carried by the smallest rotation
		// that accounts for how its tangent moved. Everything else follows from it along the rope,
		// so this single step is what makes the whole material frame temporally coherent.
		ref_normals[0] = rope_orthonormalize(rope_transport(ref_normals[0], ref_tangents[0], tangent), tangent);
	} else {
		const Vector3 reference = Math::abs(tangent.y) > (real_t)0.9 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
		ref_normals[0] = rope_orthonormalize(reference.cross(tangent), tangent);
	}
	ref_tangents[0] = tangent;

	for (uint32_t e = 1; e < segments; e++) {
		Vector3 next = positions[e + 1] - positions[e];
		if (next.length_squared() < (real_t)ROPE_EPSILON) {
			next = ref_tangents[e - 1];
		} else {
			next.normalize();
		}

		// Parallel transport along the rope. Because every segment's frame is derived from its
		// predecessor this way, adjacent frames differ by no rotation about the tangent at all --
		// which is what lets twist be a single scalar per segment rather than a frame comparison.
		ref_normals[e] = rope_orthonormalize(rope_transport(ref_normals[e - 1], ref_tangents[e - 1], next), next);
		ref_tangents[e] = next;
	}

	ref_frame_valid = true;
}

Vector<Vector3> JoltRope3D::get_points_interpolated(float p_fraction) const {
	const uint32_t count = positions.size();

	if (!render_positions_valid || render_positions.size() != count) {
		return get_points();
	}

	Vector<Vector3> result;
	result.resize((int)count);
	Vector3 *write = result.ptrw();

	const real_t fraction = CLAMP((real_t)p_fraction, (real_t)0.0, (real_t)1.0);
	for (uint32_t i = 0; i < count; i++) {
		write[i] = render_positions[i].lerp(positions[i], fraction);
	}

	return result;
}

void JoltRope3D::reset_interpolation() {
	render_positions_valid = false;
}

Vector<Vector3> JoltRope3D::get_point_normals() const {
	const uint32_t count = positions.size();

	Vector<Vector3> result;
	if (count < 2 || ref_normals.size() != count - 1) {
		return result;
	}

	result.resize((int)count);
	Vector3 *write = result.ptrw();

	// The drawn frame is the *material* frame -- reference rolled by however much the rope is twisted
	// there -- so the surface shows the torsion the solver is carrying rather than an independent
	// guess at it.
	const bool twisted = twist_angles.size() == count - 1 && twist_compliance < ROPE_TWIST_DISABLED_COMPLIANCE;

	auto material = [&](uint32_t p_segment) {
		return twisted
				? rope_roll(ref_normals[p_segment], ref_tangents[p_segment], (real_t)twist_angles[p_segment])
				: ref_normals[p_segment];
	};

	// Frames live on segments, rings live on particles. An interior particle takes the sum of the two
	// segments meeting there, which is the cheap stand-in for slerping between them and is exact
	// enough for a chain this smooth; the ends take their single neighbour.
	write[0] = material(0);
	write[count - 1] = material(count - 2);

	for (uint32_t i = 1; i < count - 1; i++) {
		Vector3 tangent = positions[i + 1] - positions[i - 1];
		if (tangent.length_squared() < (real_t)ROPE_EPSILON) {
			write[i] = material(i - 1);
			continue;
		}
		tangent.normalize();
		write[i] = rope_orthonormalize(material(i - 1) + material(i), tangent);
	}

	return result;
}

// Works out, for every locked attachment, what roll the thing holding the rope is currently asking
// for. Runs after `_update_frames()`, because it is measured against this frame's reference.
void JoltRope3D::_update_twist_targets() {
	const uint32_t count = positions.size();
	const uint32_t segments = count - 1;

	for (KeyValue<int, Attachment> &E : attachments) {
		Attachment &attachment = E.value;
		attachment.twist_locked = false;

		if (!attachment.lock_twist || attachment.mode == ATTACH_NONE) {
			continue;
		}

		const int index = E.key;
		if (index < 0 || index >= (int)count) {
			continue;
		}

		// A lock lives on a segment, not a particle. The last particle has no segment of its own, so
		// it takes the one arriving at it.
		const uint32_t segment = (index >= (int)segments) ? segments - 1 : (uint32_t)index;
		attachment.twist_segment = (int)segment;

		const Vector3 &tangent = ref_tangents[segment];
		const Vector3 &reference = ref_normals[segment];

		if (!attachment.twist_captured) {
			// Fasten the rope exactly as it stands. Anything else would snap it to some canonical
			// roll the moment the lock was switched on.
			const Vector3 material = rope_roll(reference, tangent, (real_t)twist_angles[segment]);
			attachment.locked_normal = attachment.holder_basis.transposed().xform(material);
			attachment.twist_target = (real_t)twist_angles[segment];
			attachment.twist_captured = true;
		}

		Vector3 wanted = attachment.holder_basis.xform(attachment.locked_normal);
		wanted -= tangent * wanted.dot(tangent);
		if (wanted.length_squared() < (real_t)ROPE_EPSILON) {
			// The holder has turned its locked direction onto the rope's own axis, so there is no
			// roll left to read there. Hold last frame's target rather than snapping to an arbitrary
			// one.
			attachment.twist_locked = true;
			continue;
		}
		wanted.normalize();

		const real_t angle = Math::atan2(tangent.dot(reference.cross(wanted)), reference.dot(wanted));

		// Unwrap onto the running total. `angle` only ever comes back in (-pi, pi], so without this a
		// rope wound past half a turn would appear to spring back.
		// `Math::PI` is a double while `real_t` is a float in a single-precision build, and `wrapf()`
		// overloads on both -- so the bounds have to be narrowed explicitly or the call matches neither
		// candidate cleanly.
		const real_t half_turn = (real_t)Math::PI;
		const real_t wrapped = Math::wrapf(attachment.twist_target, -half_turn, half_turn);
		attachment.twist_target += Math::wrapf(angle - wrapped, -half_turn, half_turn);
		attachment.twist_locked = true;
	}
}

// Solves the twist chain exactly, with the Thomas algorithm.
//
// Twist is one-dimensional, has no unilateral cases and -- in this reduced formulation -- does not
// feed back into the centreline, so the implicit-Euler system for it is symmetric, positive definite
// and tridiagonal. That is worth taking advantage of: a direct solve costs the same as a single
// Gauss-Seidel sweep and is *exact*, so torsional stiffness is unconditionally stable at any value
// and any timestep. Iterating instead would have been badly conditioned for no reason, because a
// rope segment's torsional inertia is minute -- of order 1e-5 kg m^2 at the default radius and mass.
void JoltRope3D::_solve_twist(float p_step) {
	const uint32_t count = positions.size();
	if (count < 2 || twist_compliance >= ROPE_TWIST_DISABLED_COMPLIANCE) {
		return;
	}

	const uint32_t segments = count - 1;
	if (ref_normals.size() != segments || twist_angles.size() != segments) {
		return;
	}

	_update_twist_targets();

	const float stiffness = 1.0f / MAX(twist_compliance, 1e-9f);
	const float inv_step_squared = 1.0f / (p_step * p_step);
	const float segment_mass = total_mass / (float)count;
	// A thin cylinder about its own axis. Floored so a massless or zero-radius rope still gives a
	// conditioned system rather than a singular one.
	const float inertia = MAX(0.5f * segment_mass * radius * radius, 1e-8f);
	const float mass_term = inertia * inv_step_squared;
	const float retain = MAX(0.0f, 1.0f - twist_damping * p_step);

	for (uint32_t e = 0; e < segments; e++) {
		twist_predicted[e] = twist_angles[e] + p_step * twist_velocities[e] * retain;
	}

	for (uint32_t e = 0; e < segments; e++) {
		const Attachment *lock = nullptr;
		for (const KeyValue<int, Attachment> &E : attachments) {
			if (E.value.twist_locked && E.value.twist_segment == (int)e) {
				lock = &E.value;
				break;
			}
		}

		float lower = (e > 0) ? -stiffness : 0.0f;
		float upper = (e + 1 < segments) ? -stiffness : 0.0f;
		float row_mass = mass_term;
		float predicted = twist_predicted[e];

		if (lock != nullptr) {
			// Where the holder's own rotation has put this segment, ignoring the rope.
			predicted = (float)lock->twist_target;

			const ColliderInfo *info = (lock->collider >= 0) ? &colliders[lock->collider] : nullptr;
			const real_t inv_axis_inertia = (info != nullptr && info->dynamic)
					? ref_tangents[e].dot(info->inv_inertia.xform(ref_tangents[e]))
					: (real_t)0.0;

			if (inv_axis_inertia > (real_t)ROPE_EPSILON) {
				// A dynamic holder is just another element of the twist chain, only far heavier. Its
				// inertia goes into the row rather than the row being pinned outright, which is what
				// keeps the coupling implicit and therefore stable at any stiffness.
				//
				// Pinning the row and handing the holder the leftover torque afterwards is an
				// explicit spring, and it diverges exactly as one would expect: at zero compliance a
				// hanging load spun up five orders of magnitude past its initial rate within seconds.
				row_mass = (float)(1.0 / inv_axis_inertia) * inv_step_squared;
			} else {
				// Static, kinematic, or spinning about an axis it has no inertia around: genuinely
				// immovable, so this is a Dirichlet row and the sweep below carries it into its
				// neighbours as a known value with no special casing at all.
				lower = 0.0f;
				upper = 0.0f;
				row_mass = 1.0f;
			}
		}

		const float diagonal = row_mass - lower - upper;
		const float rhs = row_mass * predicted;

		const float pivot = diagonal - lower * ((e > 0) ? twist_scratch_c[e - 1] : 0.0f);
		if (Math::abs(pivot) < (float)ROPE_EPSILON) {
			return;
		}

		twist_scratch_c[e] = upper / pivot;
		twist_scratch_d[e] = (rhs - lower * ((e > 0) ? twist_scratch_d[e - 1] : 0.0f)) / pivot;
	}

	for (uint32_t s = segments; s-- > 0;) {
		const float next = (s + 1 < segments) ? twist_angles[s + 1] : 0.0f;
		const float solved = twist_scratch_d[s] - twist_scratch_c[s] * next;
		twist_velocities[s] = (solved - twist_angles[s]) / p_step;
		twist_angles[s] = solved;
	}

	// The rope and its holder share a degree of freedom at a lock, so once the chain is solved the
	// holder has to be brought to the answer. The rope cannot rotate a Jolt body directly, so as
	// everywhere else that becomes a velocity change -- and because the holder's inertia was part of
	// the solve, the angle it is being asked to make up is already bounded.
	//
	// This is the whole point of the feature. Without it a load hangs there spinning freely however
	// stiff the rope is set.
	for (const KeyValue<int, Attachment> &E : attachments) {
		const Attachment &attachment = E.value;
		if (!attachment.twist_locked || attachment.collider < 0) {
			continue;
		}

		ColliderInfo &info = colliders[attachment.collider];
		if (!info.dynamic) {
			continue;
		}

		const uint32_t e = (uint32_t)attachment.twist_segment;
		const Vector3 &axis = ref_tangents[e];
		const real_t inv_axis_inertia = axis.dot(info.inv_inertia.xform(axis));
		if (inv_axis_inertia <= (real_t)ROPE_EPSILON) {
			continue;
		}

		const real_t correction = (real_t)twist_angles[e] - attachment.twist_target;
		info.angular_impulse += axis * ((real_t)(1.0 / inv_axis_inertia) * correction / (real_t)p_step);
	}
}

void JoltRope3D::_update_bounds() {
	const uint32_t count = positions.size();
	if (count == 0) {
		bounds = AABB();
		return;
	}

	bounds = AABB(positions[0], Vector3());
	for (uint32_t i = 1; i < count; i++) {
		bounds.expand_to(positions[i]);
	}
	bounds.grow_by(radius);
}

// Velocity-level constraint pass, run once after the position substeps.
//
// A positional solver that runs *after* Jolt has stepped cannot remove the velocity Jolt already
// gave a body -- it can only decide where the body should have ended up. So a load that swings out
// of reach arrives with its entire approach speed intact, and if all the rope does is push it back,
// that speed carries it straight out again on the next frame. The result is a sawtooth in the
// along-rope velocity that reads as bouncing, and it only appears under motion -- which is exactly
// why the same load hanging still looks perfect.
//
// Everything here works against the body's *real* Jolt pose rather than the substep proxy. The proxy
// is a bookkeeping device for making the position solve converge; it is never applied to Jolt, so
// the error this pass has to answer for is the one measured against the real pose.
void JoltRope3D::_solve_velocities() {
	for (const KeyValue<int, Attachment> &E : attachments) {
		const Attachment &attachment = E.value;
		const int index = E.key;

		if (!attachment.soft || index < 0 || index >= (int)positions.size()) {
			continue;
		}

		ColliderInfo &info = colliders[attachment.collider];
		if (!info.dynamic) {
			continue;
		}

		// Nothing has been written back to Jolt yet, so the body's velocity for this pass is the one
		// it will have once the positional reaction lands.
		Vector3 body_linear = info.linear_velocity + info.linear_impulse * info.inv_mass;
		Vector3 body_angular = info.angular_velocity + info.inv_inertia.xform(info.angular_impulse);

		const Vector3 lever = attachment.target - info.com;

		// The limit joint, against the immovable anchor the rope's rest length is measured from.
		// One-sided on purpose: a rope resists being pulled past its length and does nothing at all
		// about a load moving back toward the anchor, because that is a rope going slack.
		//
		// This is solved *predictively* rather than correctively. The obvious formulation -- wait for
		// the body to end up outside the limit, then remove its outward velocity and add a bias to
		// drag it back -- has two costs that are hard to tune away. The bias never converges, because
		// the rope cannot move a Jolt body and so a fresh error appears every frame, and it settles
		// at whatever offset makes the correction match the drift: measured at two centimetres past
		// the limit on a swinging load, with the rope's last particle trailing the body it was tied
		// to by exactly that much. Worse, a term that permanently drags on a body is a term that
		// permanently does work on it, which quietly drained a freely rotating load.
		//
		// Asking instead where the body will *be* removes both. Jolt integrates in a straight line,
		// so requiring that the next step land the attachment point back on the limit sphere,
		//
		//     |d + v h| = limit
		//
		// and solving for the radial part of `v` with the tangential part left alone gives
		//
		//     n.v = (limit^2 - L^2 - |v_t|^2 h^2) / (2 L h)
		//
		// which is a target rate, not a correction. It handles slack for free -- while the rope has
		// length to spare the target is positive and this does nothing -- and it brakes a load
		// arriving fast over however many frames it takes to stop exactly at full extension, instead
		// of letting it overshoot and then hauling it back.
		if (attachment.limit_anchor < 0 || !inextensible || attachment.limit_constraint != nullptr) {
			continue;
		}

		const Vector3 span = attachment.target - positions[attachment.limit_anchor];
		const float span_length = (float)span.length();
		if (span_length < ROPE_EPSILON) {
			continue;
		}

		const Vector3 span_normal = span / span_length;
		const Vector3 point_velocity = body_linear + body_angular.cross(lever);
		const real_t separating = point_velocity.dot(span_normal);

		const Vector3 tangential = point_velocity - span_normal * separating;
		const real_t limit_squared = (real_t)attachment.limit_distance * (real_t)attachment.limit_distance;
		const real_t travel = tangential.length_squared() * (real_t)frame_step * (real_t)frame_step;

		real_t target = (limit_squared - (real_t)span_length * (real_t)span_length - travel) /
				((real_t)2.0 * (real_t)span_length * (real_t)frame_step);

		// A body that is already well outside its limit -- teleported, or spawned there -- would
		// otherwise be asked for whatever speed clears the whole error in one frame. Reeling it in at
		// a bounded rate is the only part of this that is a choice rather than a consequence.
		//
		// Note that raising this does *not* help a body whose load arrives through another physics
		// constraint. That constraint is re-solved from scratch every step and overwrites whatever the
		// rope wrote, so no reel-in rate can win against it. `ROPE_ATTACHMENT_SOLVER_LIMIT` is the
		// answer there, and it bypasses this path entirely.
		target = MAX(target, -(real_t)ROPE_MAX_REEL_IN_SPEED);

		if (separating <= target) {
			continue;
		}

		const Vector3 lever_cross = lever.cross(span_normal);
		const float span_w = info.inv_mass + (float)lever_cross.dot(info.inv_inertia.xform(lever_cross));
		if (span_w <= 0.0f) {
			continue;
		}

		const Vector3 impulse = span_normal * (-(float)(separating - target) / span_w);

		info.linear_impulse += impulse;
		info.angular_impulse += lever.cross(impulse);
	}
}

// Relaxes and bounds the *positional* half of the reaction, before `_solve_velocities()` adds the
// velocity half on top.
void JoltRope3D::_finalize_reactions() {
	for (ColliderInfo &info : colliders) {
		if (!info.dynamic) {
			continue;
		}

		info.linear_impulse *= (real_t)ROPE_REACTION_RELAXATION;
		info.angular_impulse *= (real_t)ROPE_REACTION_RELAXATION;

		if (max_reaction_impulse <= 0.0f) {
			continue;
		}

		const real_t magnitude = info.linear_impulse.length();
		if (magnitude <= (real_t)max_reaction_impulse) {
			continue;
		}

		// Both accumulators are built from the same constraint impulses, so they are scaled together
		// -- clamping the translation alone would leave a spin behind that no longer corresponds to
		// any force the rope is applying.
		const real_t scale = (real_t)max_reaction_impulse / magnitude;
		info.linear_impulse *= scale;
		info.angular_impulse *= scale;
	}
}

// Hands the accumulated constraint impulse back to Jolt.
//
// The rope has no authority to move a Jolt body directly -- Jolt has already finished stepping by
// the time the rope runs -- so all it can do is change velocity. An impulse converts to exactly that
// with no fudge factor: divide by the mass. Everything that decides *how much* has happened by now,
// in the two functions above.
void JoltRope3D::_apply_reactions() {
	JPH::BodyInterface &body_iface = space->get_body_iface();

	for (ColliderInfo &info : colliders) {
		const Vector3 linear_impulse = info.linear_impulse;
		const Vector3 angular_impulse = info.angular_impulse;

		info.position_delta = Vector3();
		info.rotation_delta = Vector3();
		info.linear_impulse = Vector3();
		info.angular_impulse = Vector3();

		if (!info.dynamic) {
			continue;
		}

		if (linear_impulse.length_squared() < (real_t)ROPE_EPSILON && angular_impulse.length_squared() < (real_t)ROPE_EPSILON) {
			continue;
		}

		JPH::Body *jolt_body = space->try_get_jolt_body(info.body_id);
		if (jolt_body == nullptr || !jolt_body->IsDynamic()) {
			continue;
		}

		const Vector3 linear = linear_impulse * info.inv_mass;
		const Vector3 angular = info.inv_inertia.xform(angular_impulse);

		body_iface.ActivateBody(info.body_id);

		jolt_body->SetLinearVelocity(jolt_body->GetLinearVelocity() + to_jolt(linear));
		jolt_body->SetAngularVelocity(jolt_body->GetAngularVelocity() + to_jolt(angular));
	}
}

/* -------------------------------------------------------------------------- */
/* Step                                                                       */
/* -------------------------------------------------------------------------- */

void JoltRope3D::step(float p_step) {
	const uint32_t count = positions.size();
	if (count < 2 || space == nullptr || p_step <= 0.0f) {
		return;
	}

	frame_step = p_step;

	// Snapshot the pose the renderer has been showing before it is overwritten, so this step can be
	// drawn as the interval it actually is rather than as a jump to its end.
	render_positions.resize(count);
	for (uint32_t i = 0; i < count; i++) {
		render_positions[i] = positions[i];
	}
	render_positions_valid = true;

	colliders.clear();
	collider_lookup.clear();

	_update_attachment_targets();
	_update_gravity();

	if (lra_dirty) {
		_update_lra();
	}

	if (collision_enabled) {
		// Contacts are gathered against the rope's current pose, so the bounds used to reject the
		// whole rope in one broadphase test must be current too.
		_update_bounds();
		_gather_contacts(p_step);
	} else {
		contacts.clear();
	}

	const float substep = p_step / (float)substeps;

	// Constraint order matters more than it looks. Everything that enforces length runs *last*,
	// because every other constraint here violates it: a contact push is roughly perpendicular to
	// the rope and so lengthens both adjacent segments, and bending pushes particles apart to
	// straighten the chain. If those ran afterwards, the stretch they introduce would survive to
	// `_update_velocities()` and get baked in as momentum, and the rope would visibly creep longer
	// every time it touched anything.
	//
	// Attachments run first so that bending, contacts and LRA all measure against pin positions
	// that are already correct for this substep. Resolving contacts one constraint-block "early"
	// costs nothing in practice: `_gather_contacts()` collects speculative contacts with a margin
	// (`ROPE_SPECULATIVE_MARGIN`), so they are resolved before real penetration, and the distance
	// correction runs mostly along the rope, which is tangential to a contact plane.
	for (int s = 0; s < substeps; s++) {
		_solve_damping(substep);
		_integrate(substep);
		_solve_attachments(substep);
		_solve_bending(substep);
		_solve_collisions(substep);
		// The long-range pass clamps each particle's distance from its nearest anchor, which is a
		// coarse global correction: with two anchors, particles either side of the midpoint are
		// pulled toward *different* anchors, and if those anchors are further apart than the rope
		// is long it will tear the chain open at the seam. So it runs before the distance solve,
		// which heals the seam, and the per-segment strain limit gets the last word -- that one is
		// a hard upper bound on every segment, so the total length can never exceed the rest length
		// no matter what the passes above did.
		_solve_lra();
		_solve_distance(substep);
		_solve_strain_limit();
		_update_velocities(substep);
	}

	// Unconditional: attachments to dynamic bodies always transmit force now, and collision
	// reactions only ever accumulate when `two_way_coupling` is on, so there is nothing to gate.
	//
	// Relax and bound the positional half first, then let the velocity pass add its own impulses on
	// top uncapped -- that half can only ever bring a body to rest against the constraint, never push
	// it past, so bounding it would stop a heavy load short and let it sink further in instead.
	// The material frame is built from the final pose, and twist is solved against it -- so both run
	// after the substep loop, but before the reactions are handed to Jolt, because a locked twist is
	// one of the things the rope has to push back with.
	_update_frames();

	_finalize_reactions();
	_solve_twist(p_step);
	_solve_velocities();
	_apply_reactions();

	_update_bounds();
}
