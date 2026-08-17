# RopeBody3D — a first-class rope physics body for Jolt

## Context

There is no rope, cable, or chain physics anywhere in this fork (verified by grep across
`modules/`, `servers/`, `scene/`). Today the only way to build a rope in Godot is a chain of
`RigidBody3D` + `PinJoint3D`, which is both wrong and slow: sequential-impulse solvers converge
poorly on long serial chains (visibly stretchy, jittery), and a 30-segment rope costs 30
broadphase proxies, 30 islands to merge, and 30 constraint rows. Ten of those is dead on a Quest.

The goal is a single body type that covers all four rope use cases you named — grappling/swinging,
towing/winching/cranes, decorative rigging, and nets/bridges that hold weight — with real
collision, attachment at arbitrary points, and a CPU cost that scales from Quest to desktop.

### Decisions already made

- **Jolt only.** GodotPhysics3D is not a target.
- **In-engine, in `modules/jolt_physics/`** — not a GDExtension. A GDExtension cannot reach Jolt
  (statically linked, hidden symbols), which would force hand-writing capsule-vs-{sphere, box,
  capsule, cylinder, convex, trimesh, heightmap} narrowphase from `shape_get_data` Variants. In-engine,
  `JPH::NarrowPhaseQuery::CollideShape` does all of it for free.
- **Segment/edge collision**, per-rope **two-way coupling toggle**, must scale mobile↔desktop.
- **Phase 1 first, then review.** Phase 2 scope is listed at the end but not built yet.

### Why not `JPH::SoftBody`

Verified in `thirdparty/jolt_physics/Jolt/Physics/SoftBody/`: soft-body collision is **one plane per
vertex** (`SoftBodyVertex.h:37-40` — `mCollisionPlane`, `mCollidingShapeIndex`, a single
`mLargestPenetration` winner per vertex per step). There is no segment test, so a rope leaks through
anything thinner than the particle spacing. It also has no mid-rope body attachment, no runtime
length change, and no rope-to-rope attach. The Cosserat rod constraints
(`SoftBodySharedSettings.h:312`/`:331`, present and compiled) add torsion but do not fix collision.

So: a purpose-built XPBD solver, using Jolt only for collision queries and rigid-body coupling.

---

## Simulation design

**XPBD with substepping.** Per Macklin et al. 2019 ("Small Steps in Physics Simulation"), many
substeps × 1 iteration converges far better than 1 step × many iterations at equal cost — which is
exactly what an inextensible rope needs. Three things make it stiff without brute force:

1. **Substepping** (`substeps`, default 4; 8 on desktop) — the dominant win.
2. **Symmetric Gauss-Seidel** — forward then backward sweep over the distance constraints, removing
   the directional bias that makes a one-way sweep sag at the far end. Free.
3. **Long-Range Attachment (LRA)** — for each particle, clamp its distance from the nearest anchor to
   `accumulated_rest_length`. One `O(n)` pass that enforces inextensibility *globally*. This is what
   makes a grappling hook snap taut instantly, and it is the mechanism by which the rope "indirectly
   acts as a limit joint."

Per physics frame, in `JoltSpace3D::_pre_step()` (`modules/jolt_physics/spaces/jolt_space_3d.cpp:66`):

```
1. Refresh anchors from attached body/node transforms.
2. Broadphase ONCE: rope AABB grown by radius + max travel
     -> BroadPhaseQuery -> candidate JPH::BodyID set
     -> lock, cache per collider: COM transform, inv_mass, inv_inertia,
        friction, restitution, motion type, leaf shapes.
3. Narrowphase ONCE: per segment, a JPH::CapsuleShape from p[i] to p[i+1]
     -> NarrowPhaseQuery::CollideShape against the cached candidates
     -> store contacts {segment, t, normal, depth, collider, local_point}, capped per segment.
4. for substep in 0..N:                      // h = dt / N
     a. integrate      vel += g*h; vel *= damp; prev = pos; pos += vel*h
     b. distance       XPBD compliance, forward sweep + backward sweep
     c. bending        optional angular stiffness
     d. LRA            O(n) clamp from each anchor
     e. attachments    static pin / body attach, generalized inverse mass
     f. collision      re-evaluate cached contact planes against the collider's CURRENT
                       transform (cheap, handles moving platforms), project the segment out,
                       distribute the correction to p[i]/p[i+1] weighted by (1-t, t)
     g. vel = (pos - prev) / h;  tangential friction on contacted points
5. Apply accumulated reaction impulses to bodies (clamped) if two-way is on.
6. Recompute AABB; mark render data dirty.
```

**The critical performance property**: collision *detection* runs once per frame; only *resolution*
runs per substep, against a cached plane set. That is what keeps 8 substeps affordable on a Quest.

Two Jolt details worth using deliberately:

- `CollideShapeSettings::mMaxSeparationDistance` (`Collision/CollideShape.h:100`) lets us gather
  *speculative* contacts slightly before they penetrate, so one detection pass stays valid across all
  substeps of the frame.
- `mActiveEdgeMode = CollideOnlyWithActive` (`CollideSettingsBase.h:77`), or
  `CollideShapeWithInternalEdgeRemoval` (`NarrowPhaseQuery.h:56`), stops the rope catching on the
  internal edges of trimesh colliders as it slides — a very visible artifact otherwise.

**Two-way coupling** accumulates impulses per contacted collider across substeps and applies them
once at frame end via `JPH::Body::AddImpulse`, weighted by generalized inverse mass
`w = inv_mass + (r × n)·I⁻¹·(r × n)` so heavy bodies barely move, and clamped by
`max_reaction_impulse` so a stiff rope can't blow up a light body. Per-rope flag, default off.

---

## Files

### New — solver and Jolt integration

| File | Contents |
|---|---|
| `modules/jolt_physics/objects/jolt_rope_3d.h/.cpp` | `JoltRope3D : JoltObject3D`. Particle arrays (`pos`, `prev_pos`, `vel`, `inv_mass`, `rest_len`), the XPBD solve, the contact cache, attachments, `pre_step()`/`post_step()`. The bulk of the work. |

`objects/*.cpp` is wildcarded in `modules/jolt_physics/SCsub:173` — **no build-system edit needed.**

### New — scene node

| File | Contents |
|---|---|
| `scene/3d/physics/rope_body_3d.h/.cpp` | `RopeBody3D : MeshInstance3D`. Owns the RID, builds/streams the visual mesh, exposes properties and attachments. |

`scene/3d/physics/SCsub` is also a wildcard — no edit.

### Modified

| File | Change |
|---|---|
| `servers/physics_3d/physics_server_3d.h` | `rope_*` declarations + `RopeParameter`/`RopeFlag` enums + `VARIANT_ENUM_CAST`. **Give them default no-op bodies, not `= 0`.** |
| `servers/physics_3d/physics_server_3d.cpp` | `ClassDB::bind_method` + `BIND_ENUM_CONSTANT` for the new family. |
| `servers/physics_3d/physics_server_3d_wrap_mt.h` | One `FUNCn`/`FUNCnRC`/`FUNCRID` macro per method — required even with defaults, or ropes break under `physics/3d/run_on_separate_thread`. |
| `modules/jolt_physics/jolt_physics_server_3d.h/.cpp` | `RID_PtrOwner<JoltRope3D, true> rope_owner` next to `soft_body_owner` (`:53`); real implementations; a `rope_owner.owns()` branch in `free_rid` (`:1584`). |
| `modules/jolt_physics/spaces/jolt_space_3d.h/.cpp` | `SelfList<JoltRope3D>::List active_rope_list` + enqueue/dequeue, mirroring `body_call_queries_list` (`:60`). Step the ropes from `_pre_step` (`:66`); flush reaction impulses in `_post_step` (`:99`). |
| `scene/register_scene_types.cpp` | Include next to `soft_body_3d.h` (`:358`); `GDREGISTER_CLASS(RopeBody3D)` next to `SoftBody3D` (`:727`). Both inside `#ifndef PHYSICS_3D_DISABLED`. |
| `doc/classes/` | `RopeBody3D.xml` + regenerated `PhysicsServer3D.xml`. |

**On the default no-op bodies**: `PhysicsServer3D` currently has no non-pure virtuals, so this is a
deliberate departure. It buys us zero edits to `physics_server_3d_dummy.h`,
`physics_server_3d_extension.h/.cpp`, and `modules/godot_physics_3d/` — roughly 300 lines of
boilerplate across four files that would exist purely to say "not supported". The Jolt server
overrides them for real; every other backend inherits a silent no-op. If a rope is used with a
non-Jolt backend, `RopeBody3D` reports it through `get_configuration_warnings()` rather than
spamming errors per frame.

---

## Rendering

`RopeBody3D` derives from `MeshInstance3D` (same choice `SoftBody3D` makes, and for the same reason)
so material slots, LOD, shadow casting, and visibility ranges come free. Three modes:

- **`RENDER_TUBE`** — generate an `ArrayMesh` tube once (`rings × radial_segments`), then rewrite
  positions and normals per frame with `RS::mesh_surface_update_vertex_region`. This is exactly
  `SoftBody3D`'s proven path (`scene/3d/physics/soft_body_3d.cpp:42-106`). Two non-obvious
  requirements copied from there: the surface **must** set `ARRAY_FLAG_USE_DYNAMIC_UPDATE` and clear
  `ARRAY_FLAG_COMPRESS_ATTRIBUTES` (`:502-503`), and normals **must** be octahedron-encoded into a
  `uint32` (`:96-102`) or they come out garbage. Supports a `Curve` radius profile and end caps.

- **`RENDER_RIBBON`** — the cheap mode. 2 vertices per ring instead of `radial_segments`, billboarded
  **in the vertex shader**, not on the CPU. This is not just a perf choice: you ship stereo/multiview
  (`f580c693b3`, `5e8e7567aa`), and a CPU-billboarded ribbon is computed for one camera, so it is
  visibly wrong in the other eye. The shader uses `INV_VIEW_MATRIX` and `EYE_OFFSET` to orient
  per-eye, which is both correct in VR and cheaper than the CPU path. Each vertex carries a side sign
  (`-1`/`+1`) and the segment tangent; the CPU only writes the centerline. Ships with a built-in
  `ShaderMaterial` default that users can replace.

- **`RENDER_NONE`** — simulation only. `get_points()` / `get_point_position(i)` drive whatever visual
  you want.

---

## Phase 1 deliverable

Everything above, specifically:

- Particles, XPBD distance constraints with symmetric sweep, LRA inextensibility, substepping
- Capsule-per-segment collision via `NarrowPhaseQuery::CollideShape`, contacts cached per frame
- Friction and restitution at contacts
- Static pins and body attachments at **any** particle index (not just the ends)
- Two-way coupling behind a per-rope flag — cheap to add once the contact cache carries body IDs
- The `rope_*` server API and the Jolt implementation
- `RopeBody3D` with tube + ribbon + none render modes
- Registration and docs

### Node API

```
Properties: point_count, segment_length, radius, total_mass, substeps,
            stretch_compliance, bend_compliance, linear_damping, drag, gravity_scale,
            friction, restitution, collision_layer, collision_mask,
            inextensible, two_way_coupling, max_reaction_impulse,
            render_mode, radial_segments, radius_curve,
            attach_start (NodePath), attach_end (NodePath)

Methods:    get_point_count(), get_point_position(i), get_point_velocity(i), get_points(),
            pin_point(i, bool), is_point_pinned(i),
            attach_point(i, node_or_body, local_offset), detach_point(i),
            apply_point_impulse(i, impulse), reset()
```

### Deferred to Phase 2

Winching / runtime length change (continuous spooling, so the end segment's rest length varies in
`[min, segment_length]` and spawns a particle on overflow rather than popping) · breaking
attachments + `get_tension()` · sleeping (the big win for many decorative ropes) · rope-to-rope
attachment for nets and rigging · self-collision via spatial hash · CCD for fast segments (a fired
grappling hook) · editor gizmo for pin handles · distance-based quality LOD.

---

## Verification

Build: `scons platform=linuxbsd target=editor -j$(nproc)` (from `MikesBuildCommands.txt`).
Make sure the project's `physics/3d/physics_engine` is `Jolt Physics` — note the editor writes that
into every new project (`editor/editor_node.cpp:8350`), but the engine-level default is
`GodotPhysics3D`, so an older project may need it set explicitly.

A test scene exercising each decided requirement:

1. **Hangs and drapes** — rope pinned at one end, 32 particles. Settles without jitter, does not stretch.
2. **Segment collision (the explicit requirement)** — swing a rope over a thin static plate whose
   thickness is well under the particle spacing. It must rest on the plate, not slip through. This
   is the test `JPH::SoftBody` would fail.
3. **Grapple / limit-joint behavior** — pin one end to the world, attach the other to a falling
   `RigidBody3D`. The rope must snap taut at its rest length and swing as a pendulum, with no
   visible stretch on the taut frame. Confirms LRA.
4. **Tow cable** — both ends attached to dynamic bodies, two-way on. Drive one; the other follows
   once taut and is unaffected while slack.
5. **Net holds weight** — several ropes attached to a shared frame, two-way on, drop a box on them.
   The box must be supported, not sink through.
6. **Stereo ribbon** — run mode 2 in VR (or the editor's stereo preview) and confirm the ribbon is
   oriented correctly in both eyes. This is the regression the CPU-billboard approach would fail.
7. **Perf** — 10 ropes × 32 particles, 4 substeps. Profile `_pre_step` and confirm the rope cost.
   Compare against the same scene built from `RigidBody3D` + `PinJoint3D` chains.

Then: `godot --doctool --headless` and commit the generated XML — CI enforces this
(`.github/workflows/linux_builds.yml:252-255`). Adding methods to `PhysicsServer3D` also changes
`extension_api.json`; run `misc/scripts/validate_extension_api.sh`.

Match the fork's established conventions from `88015d6cd7`: `float` rather than `real_t` in physics
server signatures, and clang-format before committing.
