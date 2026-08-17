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

// Bending is skipped entirely at or above this compliance. A rope is floppy by default and the
// constraint would just be arithmetic with no visible effect.
constexpr float ROPE_BEND_DISABLED_COMPLIANCE = 1e6f;

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

	// Bending spans two segments. Its rest value is deliberately the *straight* distance rather
	// than the one measured from the authored pose: that pose is only where the rope spawns, not a
	// shape it should spring back to. Capturing the pose would also make this constraint able only
	// ever to pull particles apart, which stretches the rope.
	const uint32_t bend_count = count > 1 ? count - 2 : 0;
	bend_rest_lengths.resize(bend_count);
	for (uint32_t i = 0; i < bend_count; i++) {
		bend_rest_lengths[i] = rest_lengths[i] + rest_lengths[i + 1];
	}

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
		attachment.mode = ATTACH_STATIC;
		attachment.static_position = positions[p_index];
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
	attachment.mode = ATTACH_BODY;
	attachment.body_rid = p_body_rid;
	attachment.body = p_body;
	attachment.local_offset = p_local_offset;
	attachments[p_index] = attachment;

	lra_dirty = true;
}

void JoltRope3D::detach_point(int p_index) {
	if (attachments.erase(p_index)) {
		lra_dirty = true;
	}
}

void JoltRope3D::remove_all_attachments() {
	if (!attachments.is_empty()) {
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

	// Second pass: pair every soft (dynamic-body) attachment with the nearest immovable anchor. A
	// hard-pinned particle cannot move, so the distance between it and a body attachment is bounded
	// by the rest length of the rope between them -- that bound is the whole reason a rope reads as
	// a limit joint, and it is what stops a heavy body from simply out-massing the rope's own
	// particles and dragging the chain apart.
	if (cumulative_rest.size() == count) {
		for (KeyValue<int, Attachment> &E : attachments) {
			Attachment &attachment = E.value;
			attachment.limit_anchor = -1;
			attachment.limit_distance = 0.0f;

			if (!attachment.soft || E.key < 0 || E.key >= (int)count) {
				continue;
			}

			for (const KeyValue<int, Attachment> &other : attachments) {
				if (other.value.soft || other.value.mode == ATTACH_NONE) {
					continue;
				}
				if (other.key < 0 || other.key >= (int)count || other.key == E.key) {
					continue;
				}

				const float distance = Math::abs(cumulative_rest[E.key] - cumulative_rest[other.key]);
				if (attachment.limit_anchor < 0 || distance < attachment.limit_distance) {
					attachment.limit_anchor = other.key;
					attachment.limit_distance = distance;
				}
			}
		}
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

void JoltRope3D::_integrate(float p_step) {
	const uint32_t count = positions.size();

	const float damping_factor = MAX(0.0f, 1.0f - linear_damping * p_step);

	for (uint32_t i = 0; i < count; i++) {
		prev_positions[i] = positions[i];

		if (inv_masses[i] <= 0.0f) {
			continue;
		}

		velocities[i] += gravity_cache[i] * p_step;

		if (drag > 0.0f) {
			const float speed = (float)velocities[i].length();
			if (speed > ROPE_EPSILON) {
				// Quadratic drag, integrated explicitly but clamped so it can never reverse the
				// velocity it is meant to oppose.
				const float decel = MIN(drag * speed * p_step, speed);
				velocities[i] -= velocities[i] * (decel / speed);
			}
		}

		velocities[i] *= damping_factor;
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
	const uint32_t bend_count = bend_rest_lengths.size();
	if (bend_count == 0 || bend_compliance >= ROPE_BEND_DISABLED_COMPLIANCE) {
		return;
	}

	const float alpha = bend_compliance / (p_step * p_step);

	for (uint32_t i = 0; i < bend_count; i++) {
		const float w0 = inv_masses[i];
		const float w1 = inv_masses[i + 2];
		const float w = w0 + w1;
		if (w <= 0.0f) {
			continue;
		}

		const Vector3 delta = positions[i + 2] - positions[i];
		const float length = (float)delta.length();
		if (length < ROPE_EPSILON) {
			continue;
		}

		const Vector3 normal = delta / length;
		const float error = length - bend_rest_lengths[i];
		const float delta_lambda = -error / (w + alpha);

		const Vector3 correction = normal * delta_lambda;
		positions[i] -= correction * w0;
		positions[i + 2] += correction * w1;
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
	for (const KeyValue<int, Attachment> &E : attachments) {
		const Attachment &attachment = E.value;
		const int index = E.key;

		if (index < 0 || index >= (int)positions.size() || attachment.mode == ATTACH_NONE) {
			continue;
		}

		if (!attachment.soft) {
			// Hard pin: static, kinematic, or a dynamic body without two-way coupling.
			positions[index] = attachment.target;
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
		// the next substep from asking for the whole thing again.
		info.position_delta -= correction * info.inv_mass;
		info.rotation_delta -= info.inv_inertia.xform(lever.cross(correction));

		// The limit-joint constraint. Solved against the body alone, because the anchor it is
		// measured from is immovable by construction. Without it the rope can only resist through
		// its own particles, whose combined mass is usually far less than the body's -- so a heavy
		// body would keep falling, the rope would be pulled straight past its rest length, and the
		// long-range pass would tear it open in the middle.
		if (attachment.limit_anchor < 0 || !inextensible) {
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

		// The full excess is removed from the body: it is the only thing here that can move.
		const Vector3 span_correction = span_normal * (-(span_length - attachment.limit_distance));

		info.position_delta += span_correction;
		info.rotation_delta += info.inv_inertia.xform(span_lever.cross(span_correction)) / span_w;
		positions[index] += span_correction;
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

void JoltRope3D::_apply_reactions() {
	JPH::BodyInterface &body_iface = space->get_body_iface();

	for (ColliderInfo &info : colliders) {
		// The proxy already holds exactly what the rope decided this body should do over the frame,
		// so the velocity that produces that displacement is the reaction -- no mass term needed,
		// and no ambiguity.
		//
		// Deriving the reaction from the individual positional corrections instead does not work.
		// Those corrections mean two different things depending on the situation: for a taut rope
		// they converge and their sum is the real displacement, but for a slack one the end particle
		// simply drifts under gravity and is re-fetched every substep, so the same sum is mostly
		// gravity being counted over and over. Turning that into an impulse holds the body up in
		// mid-air against its own weight.
		Vector3 linear = info.position_delta / frame_step;
		Vector3 angular = info.rotation_delta / frame_step;

		info.position_delta = Vector3();
		info.rotation_delta = Vector3();

		if (!info.dynamic) {
			continue;
		}

		if (linear.length_squared() < (real_t)ROPE_EPSILON && angular.length_squared() < (real_t)ROPE_EPSILON) {
			continue;
		}

		// A stiff rope attached to a light body can otherwise ask for an unbounded change and launch
		// it across the level. The limit is expressed as an impulse, so it becomes a velocity limit
		// once divided through by the body's mass.
		if (max_reaction_impulse > 0.0f && info.inv_mass > 0.0f) {
			const float max_speed = max_reaction_impulse * info.inv_mass;
			const float speed = (float)linear.length();
			if (speed > max_speed) {
				const float ratio = max_speed / speed;
				linear *= ratio;
				angular *= ratio;
			}
		}

		JPH::Body *jolt_body = space->try_get_jolt_body(info.body_id);
		if (jolt_body == nullptr || !jolt_body->IsDynamic()) {
			continue;
		}

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
	_apply_reactions();

	_update_bounds();
}
