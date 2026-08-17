/**************************************************************************/
/*  rope_attachment_3d.h                                                  */
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

#include "scene/3d/node_3d.h"

class PhysicsBody3D;
class RopeBody3D;

// Marks the point where a rope ties on to something.
//
// Put it under the node the rope should connect to -- typically a `PhysicsBody3D` -- position it
// where the knot goes, and point `rope` at the `RopeBody3D` plus the `ratio` along that rope which
// should be held there. Because it is an ordinary `Node3D`, you place the connection point with the
// normal editor move gizmo, and it travels with its parent for free.
//
// If there is a `PhysicsBody3D` anywhere up the parent chain, the rope is genuinely attached to that
// body at this offset and (when the body is dynamic) the two pull on each other. Otherwise this is
// simply a moving pin the rope follows.
class RopeAttachment3D : public Node3D {
	GDCLASS(RopeAttachment3D, Node3D);

	NodePath rope_path;
	float ratio = 0.0f;

	ObjectID rope_id;
	bool registered = false;

	void _resolve_rope();
	void _unregister();
	void _notify_rope();

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	void set_rope(const NodePath &p_path);
	NodePath get_rope() const { return rope_path; }

	void set_ratio(float p_ratio);
	float get_ratio() const { return ratio; }

	RopeBody3D *get_rope_body() const;
	// The nearest `PhysicsBody3D` at or above this node, or null if there is none.
	PhysicsBody3D *get_attached_body() const;

	virtual PackedStringArray get_configuration_warnings() const override;

	RopeAttachment3D();
};
