/**************************************************************************/
/*  rope_body_3d_gizmo_plugin.cpp                                         */
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

#include "rope_body_3d_gizmo_plugin.h"

#include "editor/editor_undo_redo_manager.h"
#include "editor/scene/3d/node_3d_editor_plugin.h"
#include "editor/themes/editor_scale.h"
#include "scene/3d/camera_3d.h"
#include "scene/3d/physics/rope_body_3d.h"
#include "scene/resources/curve.h"

RopeBody3DGizmoPlugin::RopeBody3DGizmoPlugin() {
	create_material("rope_material", Color(0.5, 0.8, 1.0));
	create_handle_material("handles");
	create_handle_material("pin_handles", false);
}

bool RopeBody3DGizmoPlugin::has_gizmo(Node3D *p_spatial) {
	return Object::cast_to<RopeBody3D>(p_spatial) != nullptr;
}

String RopeBody3DGizmoPlugin::get_gizmo_name() const {
	return "RopeBody3D";
}

int RopeBody3DGizmoPlugin::get_priority() const {
	return -1;
}

bool RopeBody3DGizmoPlugin::is_selectable_when_hidden() const {
	return true;
}

bool RopeBody3DGizmoPlugin::can_commit_handle_on_click() const {
	// So that clicking a particle handle without dragging still reaches `commit_handle()`, which is
	// what toggles the pin.
	return true;
}

void RopeBody3DGizmoPlugin::redraw(EditorNode3DGizmo *p_gizmo) {
	RopeBody3D *rope = Object::cast_to<RopeBody3D>(p_gizmo->get_node_3d());

	p_gizmo->clear();

	if (rope == nullptr) {
		return;
	}

	// Everything a gizmo draws is in the node's local space.
	const Transform3D to_local = rope->get_global_transform().affine_inverse();

	const PackedVector3Array points = rope->get_points();
	Vector<Vector3> particles;
	particles.resize(points.size());
	for (int i = 0; i < points.size(); i++) {
		particles.write[i] = to_local.xform(points[i]);
	}

	if (particles.size() >= 2) {
		Vector<Vector3> lines;
		for (int i = 0; i + 1 < particles.size(); i++) {
			lines.push_back(particles[i]);
			lines.push_back(particles[i + 1]);
		}

		p_gizmo->add_lines(lines, get_material("rope_material", p_gizmo));
		// Makes the rope itself clickable in the viewport rather than only its mesh.
		p_gizmo->add_collision_segments(lines);
	}

	// Primary handles: the spawn curve's control points.
	Ref<Curve3D> curve = rope->get_spawn_curve();
	HashSet<int> covered;
	if (curve.is_valid() && curve->get_point_count() > 0) {
		Vector<Vector3> curve_handles;
		for (int i = 0; i < curve->get_point_count(); i++) {
			const Vector3 position = curve->get_point_position(i);
			curve_handles.push_back(position);

			const int point_index = _particle_at(rope, position);
			if (point_index >= 0) {
				covered.insert(point_index);
			}
		}
		p_gizmo->add_handles(curve_handles, get_material("handles"));
	}

	// Secondary handles: the particles, for click-to-pin. Particles a curve control point already
	// sits on are left out. Handle materials draw on top of everything with depth testing off, so
	// two handles sharing a position have no stable draw order and visibly flicker against each
	// other -- and it is always a pinned rope end, where the blue is exactly what you want to see.
	// Nothing is lost by dropping them: the picker returns the primary handle at those positions
	// anyway, `commit_handle()` forwards its clicks to this pin, and `is_handle_highlighted()`
	// gives it the pin's colour.
	if (!particles.is_empty()) {
		Vector<Vector3> pin_handles;
		Vector<int> pin_ids;
		for (int i = 0; i < particles.size(); i++) {
			if (covered.has(i)) {
				continue;
			}
			pin_handles.push_back(particles[i]);
			pin_ids.push_back(i);
		}

		if (!pin_handles.is_empty()) {
			p_gizmo->add_handles(pin_handles, get_material("pin_handles"), pin_ids, false, true);
		}
	}
}

void RopeBody3DGizmoPlugin::begin_handle_action(const EditorNode3DGizmo *p_gizmo, int p_id, bool p_secondary) {
	// Mouse-down. Until the cursor leaves the dead zone in `set_handle()` this press is still a
	// click, which `commit_handle()` turns into a pin toggle.
	drag_pending = true;
	drag_moved = false;
}

String RopeBody3DGizmoPlugin::get_handle_name(const EditorNode3DGizmo *p_gizmo, int p_id, bool p_secondary) const {
	if (p_secondary) {
		return vformat(TTR("Rope Point %d"), p_id);
	}
	return vformat(TTR("Spawn Curve Point %d"), p_id);
}

Variant RopeBody3DGizmoPlugin::get_handle_value(const EditorNode3DGizmo *p_gizmo, int p_id, bool p_secondary) const {
	RopeBody3D *rope = Object::cast_to<RopeBody3D>(p_gizmo->get_node_3d());
	ERR_FAIL_NULL_V(rope, Variant());

	if (p_secondary) {
		return rope->find_pin_at_index(p_id) >= 0;
	}

	Ref<Curve3D> curve = rope->get_spawn_curve();
	ERR_FAIL_COND_V(curve.is_null(), Variant());
	ERR_FAIL_INDEX_V(p_id, curve->get_point_count(), Variant());

	handle_origin = curve->get_point_position(p_id);
	return handle_origin;
}

void RopeBody3DGizmoPlugin::set_handle(const EditorNode3DGizmo *p_gizmo, int p_id, bool p_secondary, Camera3D *p_camera, const Point2 &p_point) {
	// A particle is a simulation output, so there is nothing to drag; the secondary set exists only
	// for the click that `commit_handle()` turns into a pin toggle.
	if (p_secondary) {
		return;
	}

	RopeBody3D *rope = Object::cast_to<RopeBody3D>(p_gizmo->get_node_3d());
	ERR_FAIL_NULL(rope);

	Ref<Curve3D> curve = rope->get_spawn_curve();
	ERR_FAIL_COND(curve.is_null());
	ERR_FAIL_INDEX(p_id, curve->get_point_count());

	if (drag_pending) {
		drag_start = p_point;
		drag_pending = false;
	}

	// Nothing moves until the press leaves the dead zone, so that the cursor jitter in an ordinary
	// click cannot nudge the curve point that is covering a particle's pin handle.
	if (!drag_moved) {
		if (drag_start.distance_to(p_point) <= DRAG_THRESHOLD * EDSCALE) {
			return;
		}
		drag_moved = true;
	}

	const Transform3D gt = rope->get_global_transform();
	const Transform3D gi = gt.affine_inverse();

	const Vector3 ray_from = p_camera->project_ray_origin(p_point);
	const Vector3 ray_dir = p_camera->project_ray_normal(p_point);

	// A plane facing the camera through the handle's original position: the standard way to turn a
	// 2D cursor into a 3D position without a depth reference.
	const Plane plane(p_camera->get_transform().basis.get_column(2), gt.xform(handle_origin));

	Vector3 intersection;
	if (!plane.intersects_ray(ray_from, ray_dir, &intersection)) {
		return;
	}

	if (Node3DEditor::get_singleton()->is_snap_enabled()) {
		intersection.snapf(Node3DEditor::get_singleton()->get_translate_snap());
	}

	curve->set_point_position(p_id, gi.xform(intersection));
}

void RopeBody3DGizmoPlugin::commit_handle(const EditorNode3DGizmo *p_gizmo, int p_id, bool p_secondary, const Variant &p_restore, bool p_cancel) {
	RopeBody3D *rope = Object::cast_to<RopeBody3D>(p_gizmo->get_node_3d());
	ERR_FAIL_NULL(rope);

	EditorUndoRedoManager *undo_redo = EditorUndoRedoManager::get_singleton();

	if (p_secondary) {
		if (p_cancel) {
			return;
		}

		_toggle_pin(rope, p_id);
		return;
	}

	Ref<Curve3D> curve = rope->get_spawn_curve();
	ERR_FAIL_COND(curve.is_null());
	ERR_FAIL_INDEX(p_id, curve->get_point_count());

	if (p_cancel) {
		curve->set_point_position(p_id, p_restore);
		return;
	}

	// The press never left the dead zone, so it was a click rather than a drag. Since primary
	// handles win the pick, this is the only chance the particle underneath gets to be pinned --
	// and at the rope's ends there is always one directly underneath.
	if (!drag_moved) {
		const int point_index = _particle_at(rope, curve->get_point_position(p_id));
		if (point_index >= 0) {
			_toggle_pin(rope, point_index);
		}
		return;
	}

	undo_redo->create_action(TTR("Set Rope Spawn Curve Point Position"));
	undo_redo->add_do_method(curve.ptr(), "set_point_position", p_id, curve->get_point_position(p_id));
	undo_redo->add_undo_method(curve.ptr(), "set_point_position", p_id, p_restore);
	undo_redo->commit_action();
}

int RopeBody3DGizmoPlugin::_particle_at(RopeBody3D *p_rope, const Vector3 &p_local_position) const {
	const PackedVector3Array points = p_rope->get_points();
	if (points.size() < 2) {
		return -1;
	}

	const Transform3D to_local = p_rope->get_global_transform().affine_inverse();

	int closest = -1;
	real_t closest_distance_squared = 0.0;
	for (int i = 0; i < points.size(); i++) {
		const real_t distance_squared = to_local.xform(points[i]).distance_squared_to(p_local_position);
		if (closest == -1 || distance_squared < closest_distance_squared) {
			closest = i;
			closest_distance_squared = distance_squared;
		}
	}

	// Half the particle spacing keeps the test independent of the rope's scale, and is tight enough
	// that a control point pulled off the rope stops claiming a particle. Genuine overlaps sit at a
	// distance of zero: both come from sampling the same curve.
	const real_t spacing = to_local.xform(points[0]).distance_to(to_local.xform(points[1]));
	const real_t tolerance = spacing * 0.5;
	if (closest_distance_squared > tolerance * tolerance) {
		return -1;
	}

	return closest;
}

void RopeBody3DGizmoPlugin::_toggle_pin(RopeBody3D *p_rope, int p_point_index) const {
	const int point_count = p_rope->get_point_count();
	if (point_count < 2) {
		return;
	}

	const float ratio = (float)p_point_index / (float)(point_count - 1);
	const bool pinned = p_rope->find_pin_at_index(p_point_index) >= 0;

	// `toggle_pin_at_ratio()` is its own inverse, so both halves of the action are the same call --
	// which also keeps the pin array's indices consistent under undo.
	EditorUndoRedoManager *undo_redo = EditorUndoRedoManager::get_singleton();
	undo_redo->create_action(pinned ? vformat(TTR("Unpin Rope Point %d"), p_point_index) : vformat(TTR("Pin Rope Point %d"), p_point_index));
	undo_redo->add_do_method(p_rope, "toggle_pin_at_ratio", ratio);
	undo_redo->add_do_method(p_rope, "update_gizmos");
	undo_redo->add_undo_method(p_rope, "toggle_pin_at_ratio", ratio);
	undo_redo->add_undo_method(p_rope, "update_gizmos");
	undo_redo->commit_action();
}

bool RopeBody3DGizmoPlugin::is_handle_highlighted(const EditorNode3DGizmo *p_gizmo, int p_id, bool p_secondary) const {
	RopeBody3D *rope = Object::cast_to<RopeBody3D>(p_gizmo->get_node_3d());
	ERR_FAIL_NULL_V(rope, false);

	int point_index = p_id;

	if (!p_secondary) {
		// A curve control point stands in for the particle it covers, which has no handle of its
		// own, so it has to show that particle's pin state.
		Ref<Curve3D> curve = rope->get_spawn_curve();
		if (curve.is_null() || p_id >= curve->get_point_count()) {
			return false;
		}

		point_index = _particle_at(rope, curve->get_point_position(p_id));
		if (point_index < 0) {
			return false;
		}
	}

	return rope->find_pin_at_index(point_index) >= 0;
}
