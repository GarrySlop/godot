/**************************************************************************/
/*  rope_attachment_3d.cpp                                                */
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

#include "rope_attachment_3d.h"

#include "core/object/class_db.h"
#include "scene/3d/physics/physics_body_3d.h"
#include "scene/3d/physics/rope_body_3d.h"

RopeAttachment3D::RopeAttachment3D() {
	set_notify_transform(true);
}

void RopeAttachment3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			// Resolved here rather than on ENTER_TREE, because the rope this points at may well sit
			// elsewhere in the scene and not be in the tree yet when this node arrives.
			_resolve_rope();
		} break;

		case NOTIFICATION_EXIT_TREE: {
			_unregister();
		} break;

		case NOTIFICATION_TRANSFORM_CHANGED: {
			_notify_rope();
		} break;
	}
}

RopeBody3D *RopeAttachment3D::get_rope_body() const {
	return ObjectDB::get_instance<RopeBody3D>(rope_id);
}

PhysicsBody3D *RopeAttachment3D::get_attached_body() const {
	for (Node *node = get_parent(); node != nullptr; node = node->get_parent()) {
		if (PhysicsBody3D *body = Object::cast_to<PhysicsBody3D>(node)) {
			return body;
		}
		// Don't walk out of this scene branch into an unrelated ancestor body.
		if (Object::cast_to<RopeBody3D>(node) != nullptr) {
			break;
		}
	}
	return nullptr;
}

void RopeAttachment3D::_resolve_rope() {
	RopeBody3D *found = rope_path.is_empty() ? nullptr : Object::cast_to<RopeBody3D>(get_node_or_null(rope_path));
	const ObjectID found_id = found != nullptr ? found->get_instance_id() : ObjectID();

	if (registered && found_id == rope_id) {
		_notify_rope();
		return;
	}

	_unregister();

	rope_id = found_id;
	if (found != nullptr) {
		found->register_attachment(this);
		registered = true;
	}

	update_configuration_warnings();
}

void RopeAttachment3D::_unregister() {
	if (!registered) {
		return;
	}

	if (RopeBody3D *rope = get_rope_body()) {
		rope->unregister_attachment(this);
	}
	registered = false;
	rope_id = ObjectID();
}

void RopeAttachment3D::_notify_rope() {
	if (RopeBody3D *rope = get_rope_body()) {
		rope->attachment_changed();
	}
}

void RopeAttachment3D::set_rope(const NodePath &p_path) {
	rope_path = p_path;

	if (is_inside_tree()) {
		_resolve_rope();
	}
	update_configuration_warnings();
}

void RopeAttachment3D::set_ratio(float p_ratio) {
	const float clamped = CLAMP(p_ratio, 0.0f, 1.0f);
	if (Math::is_equal_approx(ratio, clamped)) {
		return;
	}

	ratio = clamped;
	_notify_rope();
}

PackedStringArray RopeAttachment3D::get_configuration_warnings() const {
	PackedStringArray warnings = Node3D::get_configuration_warnings();

	if (rope_path.is_empty()) {
		warnings.push_back(RTR("Set Rope to the RopeBody3D this attachment should connect to; it does nothing otherwise."));
	} else if (is_inside_tree() && Object::cast_to<RopeBody3D>(get_node_or_null(rope_path)) == nullptr) {
		warnings.push_back(RTR("The Rope path does not point to a valid RopeBody3D."));
	}

	return warnings;
}

void RopeAttachment3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_rope", "path"), &RopeAttachment3D::set_rope);
	ClassDB::bind_method(D_METHOD("get_rope"), &RopeAttachment3D::get_rope);

	ClassDB::bind_method(D_METHOD("set_ratio", "ratio"), &RopeAttachment3D::set_ratio);
	ClassDB::bind_method(D_METHOD("get_ratio"), &RopeAttachment3D::get_ratio);

	ClassDB::bind_method(D_METHOD("get_attached_body"), &RopeAttachment3D::get_attached_body);

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "rope", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "RopeBody3D"), "set_rope", "get_rope");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ratio", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_ratio", "get_ratio");
}
