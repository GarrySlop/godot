/**************************************************************************/
/*  jolt_rope_3d.h                                                        */
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

#pragma once

#include "core/math/aabb.h"
#include "core/math/basis.h"
#include "core/math/transform_3d.h"
#include "core/math/vector3.h"
#include "core/object/object_id.h"
#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"
#include "core/templates/rid.h"
#include "core/templates/self_list.h"
#include "core/templates/vset.h"

#include "servers/physics_3d/physics_server_3d.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>

class JoltBody3D;
class JoltSpace3D;

// A rope simulated as a chain of point masses connected by XPBD distance constraints.
//
// This is deliberately NOT a `JoltObject3D`: it owns no `JPH::Body` and does not participate in
// Jolt's own solver. Jolt is used for two things only, both of which it does far better than we
// could: narrow-phase collision (a `JPH::CapsuleShape` per segment, so the rope collides along its
// whole length rather than only at its particles) and rigid-body coupling.
//
// The solver follows Macklin et al. 2019 ("Small Steps in Physics Simulation"): many substeps with
// a single constraint iteration each, rather than one step with many iterations. Combined with a
// symmetric (forward + backward) Gauss-Seidel sweep and long-range attachment constraints, this is
// what lets the rope behave as genuinely inextensible without a large iteration count.
//
// The performance-critical property is that collision *detection* runs once per frame while
// collision *resolution* runs every substep against the cached result. Re-querying per substep is
// the obvious implementation and is roughly an order of magnitude more expensive.
class JoltRope3D {
public:
	enum AttachMode {
		ATTACH_NONE,
		ATTACH_STATIC,
		ATTACH_BODY,
	};

	struct Attachment {
		AttachMode mode = ATTACH_NONE;
		Vector3 static_position;

		RID body_rid;
		JoltBody3D *body = nullptr;
		Vector3 local_offset;

		// Refreshed once per frame by `_update_attachment_targets()`.
		Vector3 target;
		int collider = -1;
		bool soft = false;

		// Optional orientation locks. Both default off, so an attachment stays a plain ball joint
		// unless asked otherwise.
		bool lock_twist = false;
		bool lock_direction = false;
		float direction_compliance = 0.0f;
		bool solver_limit = false;

		// Captured the first frame a lock is solved, in the frame of whatever holds it -- body-local
		// for a body, world for a static pin. Capturing rather than prescribing is the whole point:
		// the rope is fastened in the pose it already has, so switching a lock on never jerks it.
		Vector3 locked_normal;
		Vector3 locked_direction;
		bool twist_captured = false;
		bool direction_captured = false;

		// The frame the locks are expressed in, refreshed once per frame: the body's basis for a body
		// attachment, identity (i.e. world) for a static pin.
		Basis holder_basis;

		// Refreshed once per frame by `_update_twist_targets()`. `twist_target` is deliberately
		// unbounded -- a rope wound up three turns is not the same as one at rest, so it must not be
		// stored modulo a full turn.
		int twist_segment = -1;
		real_t twist_target = 0.0f;
		bool twist_locked = false;

		// The nearest genuinely immovable anchor along the rope, and the total rest length between
		// the two. A body attachment can never be further from that anchor than this, which is what
		// makes the rope act as a limit joint on the body it is tied to.
		int limit_anchor = -1;
		float limit_distance = 0.0f;

		// That limit, handed to Jolt as a real constraint.
		//
		// The rope itself runs after Jolt has finished stepping, so everything else it does to a body
		// is a velocity written after the fact. That is fine when the body is the load -- it holds
		// perfectly. It is not fine when the load reaches the body through *another* Jolt constraint,
		// because that constraint is re-solved from scratch on the next step and simply overwrites
		// whatever the rope wrote. Measured: a 1 kg hand tied to a rope holds its own weight
		// indefinitely, and slides 4.8 m in five seconds the moment a 16 kg body is hung off it
		// through a spring joint.
		//
		// A real constraint puts the load path in the same solver iteration as everything else acting
		// on that body, which is the only way the two can agree.
		JPH::Ref<JPH::TwoBodyConstraint> limit_constraint;
		JPH::BodyID limit_constraint_body;
		JPH::BodyID limit_constraint_anchor_body;
		Vector3 limit_constraint_point;
		Vector3 limit_constraint_offset;
		float limit_constraint_distance = -1.0f;
	};

private:
	// A collider the rope is currently touching. Cached once per frame so the substep loop can
	// re-derive contact planes from the collider's *current* transform (which keeps rope-on-moving-
	// platform correct) without paying for another narrow-phase query.
	struct ColliderInfo {
		JPH::BodyID body_id;
		Transform3D transform;
		Transform3D inv_transform;
		Vector3 com;
		Vector3 linear_velocity;
		Vector3 angular_velocity;
		Basis inv_inertia;
		float inv_mass = 0.0f;
		float friction = 0.5f;
		float restitution = 0.0f;
		bool dynamic = false;

		// Proxy state. Jolt has already finished stepping by the time the rope runs, so the real
		// body is frozen for the whole substep loop. Without somewhere to record the motion our own
		// corrections imply, every substep would re-derive the *same* full correction against a
		// body that never moved -- the constraint would never converge, and the reaction impulses
		// would compound. These two accumulate that motion so each substep sees a body that has
		// already responded to the last one; `rotation_delta` is a small-angle rotation vector,
		// which is accurate enough for the sub-degree corrections one frame produces.
		Vector3 position_delta;
		Vector3 rotation_delta;

		// The constraint impulse the rope owes this body over the frame, linear (N*s) and angular
		// (N*m*s). Deliberately separate from the positional proxy above, because the two answer
		// different questions: the proxy records where our corrections have already moved the body,
		// so the next substep does not ask for the same correction twice, while this records what
		// actually has to be handed back to Jolt.
		//
		// Accumulating impulse rather than displacement is what makes the reaction independent of
		// `substeps`. XPBD's constraint force is `lambda / h^2`, so its impulse over one substep is
		// `lambda / h`; summing that over `n` substeps of `h = H / n` gives the same total however
		// finely the frame is divided. Summing the displacements instead does not -- a constraint
		// under sustained load leaves a residual every substep, so the sum grew with the substep
		// count and the rope transmitted more force the more accurately it was solved.
		Vector3 linear_impulse;
		Vector3 angular_impulse;
		// Contacts within reach this substep, for equal-share correction.
		int active_contacts = 0;

		// Where a point that started the frame at `p_world` has been moved to by our corrections.
		Vector3 displaced(const Vector3 &p_world) const {
			return p_world + position_delta + rotation_delta.cross(p_world - com);
		}
	};

	struct Contact {
		int segment = 0;
		float t = 0.0f;
		int collider = 0;
		Vector3 local_point;
		Vector3 local_normal;
		// Accumulated normal impulse for this contact, clamped to stay non-negative across the solver
		// sweeps. This is what makes the contact group converge under iteration instead of diverging:
		// without it every sweep re-applies a full push and N contacts sharing a normal compound.
		real_t normal_lambda = 0.0f;
		// Accumulated tangential impulse, Coulomb-clamped against `normal_lambda`.
		Vector3 tangent_lambda;
	};

	RID rid;
	ObjectID instance_id;
	JoltSpace3D *space = nullptr;

	SelfList<JoltRope3D> active_list;

	// Particle state as parallel arrays. `rest_lengths`/`lambdas` are one entry shorter than the
	// particle arrays, `bend_lambdas` two entries shorter.
	LocalVector<Vector3> positions;
	LocalVector<Vector3> prev_positions;
	// The pose at the end of the *previous* physics step, kept purely so the rope can be drawn
	// interpolated. `prev_positions` above is per-substep and is the solver's own working state; this
	// one spans a whole frame, which is the interval the renderer interpolates over.
	LocalVector<Vector3> render_positions;
	bool render_positions_valid = false;
	LocalVector<Vector3> velocities;
	LocalVector<float> inv_masses;
	LocalVector<float> base_inv_masses;
	LocalVector<float> rest_lengths;
	LocalVector<float> lambdas;
	LocalVector<float> bend_lambdas;
	// Rest length from particle 0 to each particle, so the rest distance between any two is a
	// subtraction rather than a walk. Rebuilt with `rest_lengths`.
	LocalVector<float> cumulative_rest;
	// Per-particle gravity, sampled once per frame. Godot applies gravity through Areas rather than
	// through Jolt's system gravity (which `JoltSpace3D` zeroes), and `compute_gravity()` is
	// position-dependent so that point gravity works.
	LocalVector<Vector3> gravity_cache;

	// Twist: the material frame's roll about each segment's own tangent, measured from `ref_normals`
	// below, plus its rate. One scalar per segment is the whole degree of freedom -- this is the
	// reduced-coordinate rod of Bergou et al., *Discrete Elastic Rods*, where the centreline lives in
	// `positions` and the frame is a parallel-transported reference plus an angle.
	//
	// Because adjacent reference frames are parallel-transported from one another, the holonomy
	// between them is zero by construction and the twist constraint is simply the difference of two
	// neighbouring angles. What that trade gives up is bend-twist coupling: this rope stores and
	// transmits torsion, but it will not buckle into a coil the way a real over-twisted cable does.
	LocalVector<float> twist_angles;
	LocalVector<float> twist_velocities;

	// Scratch for the tridiagonal solve, kept as members so the solve allocates nothing.
	LocalVector<float> twist_scratch_c;
	LocalVector<float> twist_scratch_d;
	LocalVector<float> twist_predicted;

	// The rope's material frame: one reference normal per segment, plus the tangent it was last
	// built against. This is the frame the tube's surface is drawn with, and once twist exists it is
	// also what twist is measured from.
	//
	// The important property is that it is carried *across frames*. Deriving it fresh each frame from
	// an arbitrary perpendicular is spatially fine but temporally not: the seed rotates with the
	// rope's first tangent, so the whole surface rolls about its own axis as the rope swings, and a
	// seed that switches reference axis at a threshold makes it snap outright. Only segment 0 is
	// carried in time; the rest is parallel-transported along the rope from it, which keeps adjacent
	// segments in agreement and makes the holonomy between them zero by construction.
	LocalVector<Vector3> ref_normals;
	LocalVector<Vector3> ref_tangents;
	bool ref_frame_valid = false;

	// Long-range attachment: for each particle, the nearest anchor and the total rest length
	// between them. A particle can never be further from its anchor than that, which enforces
	// inextensibility globally in a single O(n) pass instead of relying on constraint propagation.
	LocalVector<int> lra_anchor;
	LocalVector<float> lra_distance;
	bool lra_dirty = true;

	HashMap<int, Attachment> attachments;
	VSet<RID> exceptions;

	LocalVector<ColliderInfo> colliders;
	LocalVector<Contact> contacts;
	HashMap<uint32_t, int> collider_lookup;

	AABB bounds;

	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;

	int substeps = 4;
	// Duration of the whole physics frame. The positional half of a reaction is a drift corrector,
	// not a force: it removes an error that accumulated over the frame Jolt already integrated, so it
	// is spread over that frame. Dividing by the substep instead would scale every reaction with
	// `substeps`, because a one-shot error is cleared by the first substep and the remaining ones
	// contribute nothing to average it back down.
	float frame_step = 0.0f;

	float radius = 0.05f;
	float total_mass = 1.0f;
	float stretch_compliance = 0.0f;
	float bend_compliance = 1e9f;
	float twist_compliance = 1e9f;
	float twist_damping = 0.5f;
	float linear_damping = 0.1f;
	float drag = 1.0f;
	float gravity_scale = 1.0f;
	float friction = 0.5f;
	float restitution = 0.0f;
	float max_reaction_impulse = 100.0f;
	// Requested total rest length. Zero means "use the spacing of the authored points as-is".
	float rest_length = 0.0f;

	bool inextensible = true;
	bool two_way_coupling = false;
	bool collision_enabled = true;

	void _update_masses();
	void _update_rest_lengths();
	void _update_lra();
	void _update_attachment_targets();
	void _update_limit_constraint(Attachment &p_attachment, const Attachment *p_anchor);
	void _release_limit_constraint(Attachment &p_attachment);
	void _release_all_limit_constraints();
	void _update_bounds();
	void _update_gravity();
	void _update_frames();
	void _update_twist_targets();
	void _solve_twist(float p_step);

	void _gather_contacts(float p_step);
	int _resolve_collider(const JPH::BodyID &p_body_id);

	void _solve_damping(float p_step);
	void _integrate(float p_step);
	void _solve_distance(float p_step);
	void _solve_bending(float p_step);
	void _solve_lra();
	void _solve_strain_limit();
	void _solve_attachments(float p_step);
	void _solve_direction_lock(Attachment &p_attachment, int p_index, float p_step);
	void _carry_locks(Attachment &p_attachment, int p_index, AttachMode p_mode, const RID &p_body_rid);
	void _solve_collisions(float p_step);
	// Second contact pass, after the length constraints have run. See the definition for why a taut
	// rope has to hand the residual penetration to the body rather than absorbing it.
	void _count_active_contacts();
	void _solve_collisions_anchored();
	// Velocity half of the contact constraint, to `_solve_collisions_anchored()`'s positional half.
	void _solve_contact_velocities();
	void _update_velocities(float p_step);
	void _solve_velocities();
	void _finalize_reactions();
	void _apply_reactions();

public:
	JoltRope3D();
	~JoltRope3D();

	RID get_rid() const { return rid; }
	void set_rid(const RID &p_rid) { rid = p_rid; }

	ObjectID get_instance_id() const { return instance_id; }
	void set_instance_id(ObjectID p_id) { instance_id = p_id; }

	JoltSpace3D *get_space() const { return space; }
	void set_space(JoltSpace3D *p_space);

	SelfList<JoltRope3D> *get_active_list_element() { return &active_list; }

	void set_points(const Vector<Vector3> &p_points);
	Vector<Vector3> get_points() const;
	int get_point_count() const { return (int)positions.size(); }

	Vector3 get_point_position(int p_index) const;
	void set_point_position(int p_index, const Vector3 &p_position);
	Vector3 get_point_velocity(int p_index) const;

	void set_param(PhysicsServer3D::RopeParameter p_param, float p_value);
	float get_param(PhysicsServer3D::RopeParameter p_param) const;

	void set_flag(PhysicsServer3D::RopeFlag p_flag, bool p_enabled);
	bool get_flag(PhysicsServer3D::RopeFlag p_flag) const;

	void set_simulation_substeps(int p_substeps);
	int get_simulation_substeps() const { return substeps; }

	uint32_t get_collision_layer() const { return collision_layer; }
	void set_collision_layer(uint32_t p_layer) { collision_layer = p_layer; }

	uint32_t get_collision_mask() const { return collision_mask; }
	void set_collision_mask(uint32_t p_mask) { collision_mask = p_mask; }

	void add_collision_exception(RID p_body) { exceptions.insert(p_body); }
	void remove_collision_exception(RID p_body) { exceptions.erase(p_body); }
	const VSet<RID> &get_collision_exceptions() const { return exceptions; }

	void pin_point(int p_index, bool p_pin);
	bool is_point_pinned(int p_index) const;

	void set_pin_position(int p_index, const Vector3 &p_position);
	Vector3 get_pin_position(int p_index) const;

	void attach_point_to_body(int p_index, RID p_body_rid, JoltBody3D *p_body, const Vector3 &p_local_offset);
	void detach_point(int p_index);

	void set_attachment_flag(int p_index, PhysicsServer3D::RopeAttachmentFlag p_flag, bool p_enabled);
	bool get_attachment_flag(int p_index, PhysicsServer3D::RopeAttachmentFlag p_flag) const;

	void set_attachment_param(int p_index, PhysicsServer3D::RopeAttachmentParam p_param, float p_value);
	float get_attachment_param(int p_index, PhysicsServer3D::RopeAttachmentParam p_param) const;
	void remove_all_attachments();
	// Called when a body is freed, so attachments never keep a dangling pointer.
	void detach_from_body(RID p_body_rid);

	void apply_point_impulse(int p_index, const Vector3 &p_impulse);
	void apply_central_impulse(const Vector3 &p_impulse);

	// One per particle, perpendicular to the rope there. Rotation-minimising and stable over time, so
	// a mesh built from it does not swim or snap as the rope moves.
	Vector<Vector3> get_point_normals() const;

	// The pose `p_fraction` of the way from the previous physics step to the current one. This is what
	// a physics-interpolated scene needs: everything else in it is drawn between the last two steps,
	// so a rope drawn at the current one is out of step with all of it.
	Vector<Vector3> get_points_interpolated(float p_fraction) const;
	// Drops the interpolation history, so a rope that has been teleported or respawned does not smear
	// across the jump.
	void reset_interpolation();

	AABB get_bounds() const { return bounds; }

	void step(float p_step);
};
