/**************************************************************************/
/*  rope_body_3d_gizmo_plugin.h                                           */
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

#include "editor/scene/3d/node_3d_editor_gizmos.h"

class RopeBody3D;

// Two handle sets:
//
//  - primary   -- the control points of the rope's `spawn_curve`. Drag to bend the pose the rope
//                 starts in. Few of them, and independent of `point_count`.
//  - secondary -- the rope's particles. Click to pin or unpin, the way `SoftBody3D` works.
//                 Dragging one does nothing; a particle is a simulation output, not authored data.
//
// The two sets overlap: while the editor is not simulating, the rope is posed along the spawn
// curve, so every curve control point sits exactly on a particle -- both rope ends, always.
// `EditorNode3DGizmo::handles_intersect_ray()` resolves that overlap in favour of the primary
// handle, so a press on a rope end is always reported as the curve point. To keep both gestures
// usable, a press is only a drag once it leaves a small dead zone; a press that stays inside it is
// a click, and gets forwarded to the particle underneath as a pin toggle.
//
// A covered particle also gets no handle of its own, since two handles at one position flicker
// against each other; the curve's handle shows its pin state instead.
class RopeBody3DGizmoPlugin : public EditorNode3DGizmoPlugin {
	GDCLASS(RopeBody3DGizmoPlugin, EditorNode3DGizmoPlugin);

	// Screen-space radius a press has to leave before it counts as a drag, matching the threshold
	// `Node3DEditorViewport` uses for its own click-versus-drag decisions.
	static constexpr float DRAG_THRESHOLD = 8.0f;

	// Cached at the start of a drag by `get_handle_value()`, so `set_handle()` can build a
	// view-aligned plane through where the handle actually was.
	mutable Vector3 handle_origin;

	// Click-versus-drag state for the current press on a primary handle. `begin_handle_action()` is
	// the one entry point called exactly once per press, so it is what arms `drag_pending`; the
	// first `set_handle()` after that records where the cursor started. Note that this cannot be
	// armed from `get_handle_value()`, which the viewport also calls on every motion event to build
	// its status-bar message -- doing so re-anchors the dead zone under the moving cursor, and the
	// drag never starts.
	Point2 drag_start;
	bool drag_pending = false;
	bool drag_moved = false;

	// Index of the particle a primary handle is sitting on, or -1 when the handle is off the rope
	// (an authored curve can put control points well away from it).
	int _particle_at(RopeBody3D *p_rope, const Vector3 &p_local_position) const;
	void _toggle_pin(RopeBody3D *p_rope, int p_point_index) const;

public:
	bool has_gizmo(Node3D *p_spatial) override;
	String get_gizmo_name() const override;
	int get_priority() const override;
	bool is_selectable_when_hidden() const override;
	bool can_commit_handle_on_click() const override;
	void redraw(EditorNode3DGizmo *p_gizmo) override;

	void begin_handle_action(const EditorNode3DGizmo *p_gizmo, int p_id, bool p_secondary) override;
	String get_handle_name(const EditorNode3DGizmo *p_gizmo, int p_id, bool p_secondary) const override;
	Variant get_handle_value(const EditorNode3DGizmo *p_gizmo, int p_id, bool p_secondary) const override;
	void set_handle(const EditorNode3DGizmo *p_gizmo, int p_id, bool p_secondary, Camera3D *p_camera, const Point2 &p_point) override;
	void commit_handle(const EditorNode3DGizmo *p_gizmo, int p_id, bool p_secondary, const Variant &p_restore, bool p_cancel = false) override;
	bool is_handle_highlighted(const EditorNode3DGizmo *p_gizmo, int p_id, bool p_secondary) const override;

	RopeBody3DGizmoPlugin();
};
