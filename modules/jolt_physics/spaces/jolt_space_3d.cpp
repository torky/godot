/**************************************************************************/
/*  jolt_space_3d.cpp                                                     */
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

#include "jolt_space_3d.h"

#include "../joints/jolt_joint_3d.h"
#include "../jolt_physics_server_3d.h"
#include "../jolt_project_settings.h"
#include "../misc/jolt_math_funcs.h"
#include "../misc/jolt_stream_wrappers.h"
#include "../objects/jolt_area_3d.h"
#include "../objects/jolt_body_3d.h"
#include "../shapes/jolt_custom_shape_type.h"
#include "../shapes/jolt_shape_3d.h"
#include "jolt_body_activation_listener_3d.h"
#include "jolt_contact_listener_3d.h"
#include "jolt_layers.h"
#include "jolt_physics_direct_space_state_3d.h"
#include "jolt_temp_allocator.h"

#include "core/io/file_access.h"
#include "core/os/os.h"
#include "core/os/time.h"
#include "scene/3d/node_3d.h"
#include "scene/main/node.h"
#include "core/string/print_string.h"
#include "core/variant/variant_utility.h"

#include "Jolt/Physics/Body/BodyFilter.h"
#include "Jolt/Physics/Collision/CollideShape.h"
#include "Jolt/Physics/Collision/CollideShapeVsShapePerLeaf.h"
#include "Jolt/Physics/Collision/NarrowPhaseQuery.h"
#include "Jolt/Physics/Collision/CollisionCollectorImpl.h"
#include "Jolt/Physics/Collision/Shape/CompoundShape.h"
#include "Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h"
#include "Jolt/Physics/PhysicsScene.h"

namespace {

constexpr double SPACE_DEFAULT_CONTACT_RECYCLE_RADIUS = 0.01;
constexpr double SPACE_DEFAULT_CONTACT_MAX_SEPARATION = 0.05;
constexpr double SPACE_DEFAULT_CONTACT_MAX_ALLOWED_PENETRATION = 0.01;
constexpr double SPACE_DEFAULT_CONTACT_DEFAULT_BIAS = 0.8;
constexpr double SPACE_DEFAULT_SLEEP_THRESHOLD_LINEAR = 0.1;
constexpr double SPACE_DEFAULT_SLEEP_THRESHOLD_ANGULAR = 8.0 * Math::PI / 180;
constexpr double SPACE_DEFAULT_SOLVER_ITERATIONS = 8;

} // namespace

void JoltSpace3D::_pre_step(float p_step) {
	flush_pending_objects();

	while (needs_optimization_list.first()) {
		JoltShapedObject3D *object = needs_optimization_list.first()->self();
		needs_optimization_list.remove(needs_optimization_list.first());
		object->commit_shapes(true);
	}

	steps_since_canonicalize++;
	_step_dump("pre");

	if (dump_keys_next_step > 0) {
		_dump_stable_keys(vformat("%s.%d.%d.step%d.txt", OS::get_singleton()->get_environment("JOLT_STABLE_KEY_DUMP"), OS::get_singleton()->get_process_id(), dump_canonicalize_count, 3 - dump_keys_next_step));
		dump_keys_next_step--;
	}

	contact_listener->pre_step();

	const JPH::BodyLockInterface &lock_iface = get_lock_iface();
	const JPH::BodyID *active_rigid_bodies = physics_system->GetActiveBodiesUnsafe(JPH::EBodyType::RigidBody);
	const JPH::uint32 active_rigid_body_count = physics_system->GetNumActiveBodies(JPH::EBodyType::RigidBody);

	for (JPH::uint32 i = 0; i < active_rigid_body_count; i++) {
		JPH::Body *jolt_body = lock_iface.TryGetBody(active_rigid_bodies[i]);
		JoltObject3D *object = reinterpret_cast<JoltObject3D *>(jolt_body->GetUserData());
		object->pre_step(p_step, *jolt_body);
	}
}

void JoltSpace3D::_post_step(float p_step) {
	contact_listener->post_step();
	_step_dump("post");

	while (shapes_changed_list.first()) {
		JoltShapedObject3D *object = shapes_changed_list.first()->self();
		shapes_changed_list.remove(shapes_changed_list.first());
		object->clear_previous_shape();
	}
}

JoltSpace3D::JoltSpace3D(JPH::JobSystem *p_job_system) :
		job_system(p_job_system),
		temp_allocator(new JoltTempAllocator()),
		layers(new JoltLayers()),
		contact_listener(new JoltContactListener3D(this)),
		body_activation_listener(new JoltBodyActivationListener3D()),
		physics_system(new JPH::PhysicsSystem()) {
	step_dump_path = OS::get_singleton()->get_environment("JOLT_STEP_DUMP");
	const String step_dump_range = OS::get_singleton()->get_environment("JOLT_STEP_DUMP_RANGE");
	if (!step_dump_path.is_empty() && !step_dump_range.is_empty()) {
		step_dump_first = step_dump_range.get_slicec('-', 0).to_int();
		step_dump_last = step_dump_range.contains("-") ? step_dump_range.get_slicec('-', 1).to_int() : step_dump_first;
	}
	physics_system->Init((JPH::uint)JoltProjectSettings::max_bodies, 0, (JPH::uint)JoltProjectSettings::max_body_pairs, (JPH::uint)JoltProjectSettings::max_contact_constraints, *layers, *layers, *layers);

	JPH::PhysicsSettings settings;
	settings.mBaumgarte = JoltProjectSettings::baumgarte_stabilization_factor;
	settings.mSpeculativeContactDistance = JoltProjectSettings::speculative_contact_distance;
	settings.mPenetrationSlop = JoltProjectSettings::penetration_slop;
	settings.mLinearCastThreshold = JoltProjectSettings::ccd_movement_threshold;
	settings.mLinearCastMaxPenetration = JoltProjectSettings::ccd_max_penetration;
	settings.mBodyPairCacheMaxDeltaPositionSq = JoltProjectSettings::body_pair_cache_distance_sq;
	settings.mBodyPairCacheCosMaxDeltaRotationDiv2 = JoltProjectSettings::body_pair_cache_angle_cos_div2;
	settings.mNumVelocitySteps = (JPH::uint)JoltProjectSettings::simulation_velocity_steps;
	settings.mNumPositionSteps = (JPH::uint)JoltProjectSettings::simulation_position_steps;
	settings.mMinVelocityForRestitution = JoltProjectSettings::bounce_velocity_threshold;
	settings.mTimeBeforeSleep = JoltProjectSettings::sleep_time_threshold;
	settings.mPointVelocitySleepThreshold = JoltProjectSettings::sleep_velocity_threshold;
	settings.mUseBodyPairContactCache = JoltProjectSettings::body_pair_contact_cache_enabled;
	settings.mAllowSleeping = JoltProjectSettings::sleep_allowed;
	settings.mUseLargeIslandSplitter = false;

	physics_system->SetPhysicsSettings(settings);
	physics_system->SetGravity(JPH::Vec3::sZero());
	physics_system->SetContactListener(contact_listener);
	physics_system->SetSoftBodyContactListener(contact_listener);
	physics_system->SetBodyActivationListener(body_activation_listener);

	physics_system->SetSimCollideBodyVsBody([](const JPH::Body &p_body1, const JPH::Body &p_body2, JPH::Mat44Arg p_transform_com1, JPH::Mat44Arg p_transform_com2, JPH::CollideShapeSettings &p_collide_shape_settings, JPH::CollideShapeCollector &p_collector, const JPH::ShapeFilter &p_shape_filter) {
		if (p_body1.IsSensor() || p_body2.IsSensor()) {
			JPH::CollideShapeSettings new_collide_shape_settings = p_collide_shape_settings;
			// Since we're breaking the sensor down into leaf shapes we'll end up stripping away our `JoltCustomDoubleSidedShape` decorator shape and thus any back-face collision, so we simply force-enable it like this rather than going through the trouble of reapplying the decorator.
			new_collide_shape_settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;
			JPH::SubShapeIDCreator part1, part2;
			JPH::CollideShapeVsShapePerLeaf<JPH::AnyHitCollisionCollector<JPH::CollideShapeCollector>>(p_body1.GetShape(), p_body2.GetShape(), JPH::Vec3::sOne(), JPH::Vec3::sOne(), p_transform_com1, p_transform_com2, part1, part2, new_collide_shape_settings, p_collector, p_shape_filter);
		} else {
			JPH::PhysicsSystem::sDefaultSimCollideBodyVsBody(p_body1, p_body2, p_transform_com1, p_transform_com2, p_collide_shape_settings, p_collector, p_shape_filter);
		}
	});

	physics_system->SetCombineFriction([](const JPH::Body &p_body1, const JPH::SubShapeID &p_sub_shape_id1, const JPH::Body &p_body2, const JPH::SubShapeID &p_sub_shape_id2) {
		return Math::abs(MIN(p_body1.GetFriction(), p_body2.GetFriction()));
	});

	physics_system->SetCombineRestitution([](const JPH::Body &p_body1, const JPH::SubShapeID &p_sub_shape_id1, const JPH::Body &p_body2, const JPH::SubShapeID &p_sub_shape_id2) {
		return CLAMP(p_body1.GetRestitution() + p_body2.GetRestitution(), 0.0f, 1.0f);
	});
}

JoltSpace3D::~JoltSpace3D() {
	if (direct_state != nullptr) {
		memdelete(direct_state);
		direct_state = nullptr;
	}

	if (physics_system != nullptr) {
		delete physics_system;
		physics_system = nullptr;
	}

	if (body_activation_listener != nullptr) {
		delete body_activation_listener;
		body_activation_listener = nullptr;
	}

	if (contact_listener != nullptr) {
		delete contact_listener;
		contact_listener = nullptr;
	}

	if (layers != nullptr) {
		delete layers;
		layers = nullptr;
	}

	if (temp_allocator != nullptr) {
		delete temp_allocator;
		temp_allocator = nullptr;
	}
}

void JoltSpace3D::step(float p_step) {
	stepping = true;
	last_step = p_step;
	total_time = total_time + p_step;

	_pre_step(p_step);

	const JPH::EPhysicsUpdateError update_error = physics_system->Update(p_step, 1, temp_allocator, job_system);

	if (dump_keys_next_step == 1) {
		_dump_stable_keys(vformat("%s.%d.%d.post1.txt", OS::get_singleton()->get_environment("JOLT_STABLE_KEY_DUMP"), OS::get_singleton()->get_process_id(), dump_canonicalize_count));
	}

	if ((update_error & JPH::EPhysicsUpdateError::ManifoldCacheFull) != JPH::EPhysicsUpdateError::None) {
		WARN_PRINT_ONCE(vformat("Jolt Physics manifold cache exceeded capacity and contacts were ignored. "
								"Consider increasing maximum number of contact constraints in project settings. "
								"Maximum number of contact constraints is currently set to %d.",
				JoltProjectSettings::max_contact_constraints));
	}

	if ((update_error & JPH::EPhysicsUpdateError::BodyPairCacheFull) != JPH::EPhysicsUpdateError::None) {
		WARN_PRINT_ONCE(vformat("Jolt Physics body pair cache exceeded capacity and contacts were ignored. "
								"Consider increasing maximum number of body pairs in project settings. "
								"Maximum number of body pairs is currently set to %d.",
				JoltProjectSettings::max_body_pairs));
	}

	if ((update_error & JPH::EPhysicsUpdateError::ContactConstraintsFull) != JPH::EPhysicsUpdateError::None) {
		WARN_PRINT_ONCE(vformat("Jolt Physics contact constraint buffer exceeded capacity and contacts were ignored. "
								"Consider increasing maximum number of contact constraints in project settings. "
								"Maximum number of contact constraints is currently set to %d.",
				JoltProjectSettings::max_contact_constraints));
	}

	_post_step(p_step);

	stepping = false;
}

template <typename T>
static void _call_queries_in_stable_order(typename SelfList<T>::List &p_list) {
	struct StableLess {
		bool operator()(const T *p_a, const T *p_b) const {
			const JPH::Body *a = p_a->get_jolt_body();
			const JPH::Body *b = p_b->get_jolt_body();
			if (a == nullptr || b == nullptr) {
				return a != nullptr;
			}
			return a->IsStableLess(*b);
		}
	};

	LocalVector<T *> batch;
	while (p_list.first()) {
		batch.clear();
		while (p_list.first()) {
			batch.push_back(p_list.first()->self());
			p_list.remove(p_list.first());
		}
		batch.template sort_custom<StableLess>();
		for (T *object : batch) {
			object->call_queries();
		}
	}
}

void JoltSpace3D::call_queries() {
	_call_queries_in_stable_order<JoltBody3D>(body_call_queries_list);
	_call_queries_in_stable_order<JoltArea3D>(area_call_queries_list);
}

double JoltSpace3D::get_param(PhysicsServer3D::SpaceParameter p_param) const {
	switch (p_param) {
		case PhysicsServer3D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS: {
			return SPACE_DEFAULT_CONTACT_RECYCLE_RADIUS;
		}
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_SEPARATION: {
			return SPACE_DEFAULT_CONTACT_MAX_SEPARATION;
		}
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION: {
			return SPACE_DEFAULT_CONTACT_MAX_ALLOWED_PENETRATION;
		}
		case PhysicsServer3D::SPACE_PARAM_CONTACT_DEFAULT_BIAS: {
			return SPACE_DEFAULT_CONTACT_DEFAULT_BIAS;
		}
		case PhysicsServer3D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD: {
			return SPACE_DEFAULT_SLEEP_THRESHOLD_LINEAR;
		}
		case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD: {
			return SPACE_DEFAULT_SLEEP_THRESHOLD_ANGULAR;
		}
		case PhysicsServer3D::SPACE_PARAM_BODY_TIME_TO_SLEEP: {
			return JoltProjectSettings::sleep_time_threshold;
		}
		case PhysicsServer3D::SPACE_PARAM_SOLVER_ITERATIONS: {
			return SPACE_DEFAULT_SOLVER_ITERATIONS;
		}
		default: {
			ERR_FAIL_V_MSG(0.0, vformat("Unhandled space parameter: '%d'. This should not happen. Please report this.", p_param));
		}
	}
}

void JoltSpace3D::set_param(PhysicsServer3D::SpaceParameter p_param, double p_value) {
	switch (p_param) {
		case PhysicsServer3D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS: {
			WARN_PRINT("Space-specific contact recycle radius is not supported when using Jolt Physics. Any such value will be ignored.");
		} break;
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_SEPARATION: {
			WARN_PRINT("Space-specific contact max separation is not supported when using Jolt Physics. Any such value will be ignored.");
		} break;
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION: {
			WARN_PRINT("Space-specific contact max allowed penetration is not supported when using Jolt Physics. Any such value will be ignored.");
		} break;
		case PhysicsServer3D::SPACE_PARAM_CONTACT_DEFAULT_BIAS: {
			WARN_PRINT("Space-specific contact default bias is not supported when using Jolt Physics. Any such value will be ignored.");
		} break;
		case PhysicsServer3D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD: {
			WARN_PRINT("Space-specific linear velocity sleep threshold is not supported when using Jolt Physics. Any such value will be ignored.");
		} break;
		case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD: {
			WARN_PRINT("Space-specific angular velocity sleep threshold is not supported when using Jolt Physics. Any such value will be ignored.");
		} break;
		case PhysicsServer3D::SPACE_PARAM_BODY_TIME_TO_SLEEP: {
			WARN_PRINT("Space-specific body sleep time is not supported when using Jolt Physics. Any such value will be ignored.");
		} break;
		case PhysicsServer3D::SPACE_PARAM_SOLVER_ITERATIONS: {
			WARN_PRINT("Space-specific solver iterations is not supported when using Jolt Physics. Any such value will be ignored.");
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled space parameter: '%d'. This should not happen. Please report this.", p_param));
		} break;
	}
}

JPH::BodyInterface &JoltSpace3D::get_body_iface() {
	return physics_system->GetBodyInterfaceNoLock();
}

const JPH::BodyInterface &JoltSpace3D::get_body_iface() const {
	return physics_system->GetBodyInterfaceNoLock();
}

const JPH::BodyLockInterface &JoltSpace3D::get_lock_iface() const {
	return physics_system->GetBodyLockInterfaceNoLock();
}

const JPH::BroadPhaseQuery &JoltSpace3D::get_broad_phase_query() const {
	return physics_system->GetBroadPhaseQuery();
}

const JPH::NarrowPhaseQuery &JoltSpace3D::get_narrow_phase_query() const {
	return physics_system->GetNarrowPhaseQueryNoLock();
}

JPH::ObjectLayer JoltSpace3D::map_to_object_layer(JPH::BroadPhaseLayer p_broad_phase_layer, uint32_t p_collision_layer, uint32_t p_collision_mask) {
	return layers->to_object_layer(p_broad_phase_layer, p_collision_layer, p_collision_mask);
}

void JoltSpace3D::map_from_object_layer(JPH::ObjectLayer p_object_layer, JPH::BroadPhaseLayer &r_broad_phase_layer, uint32_t &r_collision_layer, uint32_t &r_collision_mask) const {
	layers->from_object_layer(p_object_layer, r_broad_phase_layer, r_collision_layer, r_collision_mask);
}

JPH::Body *JoltSpace3D::try_get_jolt_body(const JPH::BodyID &p_body_id) const {
	return get_lock_iface().TryGetBody(p_body_id);
}

JoltObject3D *JoltSpace3D::try_get_object(const JPH::BodyID &p_body_id) const {
	const JPH::Body *jolt_body = try_get_jolt_body(p_body_id);
	if (unlikely(jolt_body == nullptr)) {
		return nullptr;
	}

	return reinterpret_cast<JoltObject3D *>(jolt_body->GetUserData());
}

JoltShapedObject3D *JoltSpace3D::try_get_shaped(const JPH::BodyID &p_body_id) const {
	JoltObject3D *object = try_get_object(p_body_id);
	if (unlikely(object == nullptr)) {
		return nullptr;
	}

	return object->as_shaped();
}

JoltBody3D *JoltSpace3D::try_get_body(const JPH::BodyID &p_body_id) const {
	JoltObject3D *object = try_get_object(p_body_id);
	if (unlikely(object == nullptr)) {
		return nullptr;
	}

	return object->as_body();
}

JoltArea3D *JoltSpace3D::try_get_area(const JPH::BodyID &p_body_id) const {
	JoltObject3D *object = try_get_object(p_body_id);
	if (unlikely(object == nullptr)) {
		return nullptr;
	}

	return object->as_area();
}

JoltSoftBody3D *JoltSpace3D::try_get_soft_body(const JPH::BodyID &p_body_id) const {
	JoltObject3D *object = try_get_object(p_body_id);
	if (unlikely(object == nullptr)) {
		return nullptr;
	}

	return object->as_soft_body();
}

JoltPhysicsDirectSpaceState3D *JoltSpace3D::get_direct_state() {
	if (direct_state == nullptr) {
		direct_state = memnew(JoltPhysicsDirectSpaceState3D(this));
	}

	return direct_state;
}

void JoltSpace3D::set_default_area(JoltArea3D *p_area) {
	if (default_area == p_area) {
		return;
	}

	if (default_area != nullptr) {
		default_area->set_default_area(false);
	}

	default_area = p_area;

	if (default_area != nullptr) {
		default_area->set_default_area(true);
	}
}

JPH::Body *JoltSpace3D::add_object(const JoltObject3D &p_object, const JPH::BodyCreationSettings &p_settings, bool p_sleeping) {
	JPH::BodyInterface &body_iface = get_body_iface();
	JPH::Body *jolt_body = body_iface.CreateBody(p_settings);
	if (unlikely(jolt_body == nullptr)) {
		ERR_PRINT_ONCE(vformat("Failed to create underlying Jolt Physics body for '%s'. "
							   "Consider increasing maximum number of bodies in project settings. "
							   "Maximum number of bodies is currently set to %d.",
				p_object.to_string(), JoltProjectSettings::max_bodies));

		return nullptr;
	}

	if (p_sleeping) {
		pending_objects_sleeping.push_back(jolt_body->GetID());
	} else {
		pending_objects_awake.push_back(jolt_body->GetID());
	}

	return jolt_body;
}

JPH::Body *JoltSpace3D::add_object(const JoltObject3D &p_object, const JPH::SoftBodyCreationSettings &p_settings, bool p_sleeping) {
	JPH::BodyInterface &body_iface = get_body_iface();
	JPH::Body *jolt_body = body_iface.CreateSoftBody(p_settings);
	if (unlikely(jolt_body == nullptr)) {
		ERR_PRINT_ONCE(vformat("Failed to create underlying Jolt Physics body for '%s'. "
							   "Consider increasing maximum number of bodies in project settings. "
							   "Maximum number of bodies is currently set to %d.",
				p_object.to_string(), JoltProjectSettings::max_bodies));

		return nullptr;
	}

	if (p_sleeping) {
		pending_objects_sleeping.push_back(jolt_body->GetID());
	} else {
		pending_objects_awake.push_back(jolt_body->GetID());
	}

	return jolt_body;
}

void JoltSpace3D::remove_object(const JPH::BodyID &p_jolt_id) {
	JPH::BodyInterface &body_iface = get_body_iface();

	if (!pending_objects_sleeping.erase_unordered(p_jolt_id) && !pending_objects_awake.erase_unordered(p_jolt_id)) {
		body_iface.RemoveBody(p_jolt_id);
	}

	body_iface.DestroyBody(p_jolt_id);

	// If we're never going to step this space, like in the editor viewport, we need to manually clean up Jolt's broad phase instead, otherwise performance can degrade when doing things like switching scenes.
	// We'll never actually have zero bodies in any space though, since we always have the default area, so we check if there's one or fewer left instead.
	if (!JoltPhysicsServer3D::get_singleton()->is_active() && physics_system->GetNumBodies() <= 1) {
		physics_system->OptimizeBroadPhase();
	}
}

void JoltSpace3D::canonicalize() {
	ERR_FAIL_COND_MSG(stepping, "Space can't be canonicalized while it is stepping.");
	flush_pending_objects();
	_canonicalize_poses();

	while (needs_optimization_list.first()) {
		JoltShapedObject3D *object = needs_optimization_list.first()->self();
		needs_optimization_list.remove(needs_optimization_list.first());
		object->commit_shapes(true);
	}

	// Bodies added above were activated, which queues a state sync that would write their round-tripped Jolt pose back into the node.
	while (body_call_queries_list.first()) {
		body_call_queries_list.remove(body_call_queries_list.first());
	}

	steps_since_canonicalize = 0;
	physics_system->ResetStableKeys();
	const uint32_t creation_order_ties = physics_system->ResetCreationOrder();
	if (creation_order_ties > 0) {
		ERR_PRINT(vformat("Jolt canonicalize found %d pair(s) of bodies identical in pose, velocity, shape and material: their order still comes from their creation history, which differs between a joiner and live clients.", creation_order_ties));
	}
	physics_system->ResetWarmStart();
	physics_system->ResetContactCache();
	contact_listener->clear_contacts();

	_canonicalize_area_overlaps();

	JPH::BodyIDVector body_ids;
	physics_system->GetBodies(body_ids);
	const JPH::BodyLockInterface &gravity_lock_iface = get_lock_iface();
	for (const JPH::BodyID &id : body_ids) {
		const JPH::Body *body = gravity_lock_iface.TryGetBody(id);
		if (body == nullptr) {
			continue;
		}
		JoltObject3D *object = reinterpret_cast<JoltObject3D *>(body->GetUserData());
		if (object == nullptr) {
			continue;
		}
		if (JoltBody3D *jolt_body = object->as_body()) {
			jolt_body->refresh_gravity();
			JPH::Body *writable = jolt_body->get_jolt_body();
			if (writable != nullptr && writable->IsDynamic()) {
				writable->ResetForce();
				writable->ResetTorque();
			}
		}
	}

	_canonicalize_sleep();

	const String dump_path = OS::get_singleton()->get_environment("JOLT_STABLE_KEY_DUMP");
	if (!dump_path.is_empty()) {
		dump_canonicalize_count++;
		_dump_stable_keys(vformat("%s.%d.%d.canon.txt", dump_path, OS::get_singleton()->get_process_id(), dump_canonicalize_count));
		dump_keys_next_step = 2;
	}
}

void JoltSpace3D::_canonicalize_sleep() {
	JPH::BodyIDVector ids;
	physics_system->GetBodies(ids);
	const JPH::BodyLockInterface &lock_iface = get_lock_iface();
	LocalVector<JPH::Body *> movable;
	for (const JPH::BodyID &id : ids) {
		JPH::BodyLockWrite lock(lock_iface, id);
		if (!lock.Succeeded()) {
			continue;
		}
		JPH::Body &body = lock.GetBody();
		if (!body.IsStatic() && body.IsInBroadPhase()) {
			movable.push_back(&body);
		}
	}
	struct StableLess {
		bool operator()(const JPH::Body *p_a, const JPH::Body *p_b) const {
			return p_a->IsStableLess(*p_b);
		}
	};
	movable.sort_custom<StableLess>();
	JPH::BodyInterface &body_iface = get_body_iface();
	for (JPH::Body *body : movable) {
		if (!body->IsActive()) {
			body_iface.ActivateBody(body->GetID());
		}
		body->ResetSleepTimer();
	}
}

void JoltSpace3D::_canonicalize_poses() {
	JPH::BodyIDVector ids;
	physics_system->GetBodies(ids);
	const JPH::BodyLockInterface &lock_iface = get_lock_iface();
	for (const JPH::BodyID &id : ids) {
		const JPH::Body *body = lock_iface.TryGetBody(id);
		if (body == nullptr) {
			continue;
		}
		JoltObject3D *object = reinterpret_cast<JoltObject3D *>(body->GetUserData());
		if (object == nullptr) {
			continue;
		}
		JoltShapedObject3D *shaped = object->as_shaped();
		if (shaped == nullptr) {
			continue;
		}
		const Node3D *node = Object::cast_to<Node3D>(object->get_instance());
		if (node == nullptr || !node->is_inside_tree()) {
			JoltBody3D *jolt_body = object->as_body();
			if (jolt_body != nullptr && jolt_body->is_kinematic()) {
				jolt_body->snap_to_kinematic_target();
			}
			continue;
		}
		const Transform3D transform = node->get_global_transform();
		Transform3D unscaled = transform;
		Vector3 exact_scale;
		JoltMath::decompose(unscaled, exact_scale);
		shaped->set_scale_exact(exact_scale);
		JoltArea3D *area = shaped->as_area();
		JoltBody3D *jolt_body = shaped->as_body();
		if (jolt_body != nullptr && jolt_body->is_kinematic()) {
			jolt_body->snap_to_kinematic_target();
		} else if (area != nullptr) {
			area->set_transform(transform);
		} else if (jolt_body != nullptr) {
			jolt_body->set_transform(transform);
		}
	}
}

void JoltSpace3D::_canonicalize_area_overlaps() {
	struct AreaEntry {
		uint32_t key;
		JPH::BodyID id;
		JoltArea3D *area;
	};
	struct Hit {
		uint32_t key;
		uint32_t other_sub;
		uint32_t self_sub;
		JPH::BodyID other_id;
		JPH::SubShapeID other_sub_id;
		JPH::SubShapeID self_sub_id;
	};
	struct AreaEntryLess {
		bool operator()(const AreaEntry &p_a, const AreaEntry &p_b) const { return p_a.key != p_b.key ? p_a.key < p_b.key : p_a.id < p_b.id; }
	};
	struct HitLess {
		bool operator()(const Hit &p_a, const Hit &p_b) const {
			if (p_a.key != p_b.key) {
				return p_a.key < p_b.key;
			}
			if (p_a.other_sub != p_b.other_sub) {
				return p_a.other_sub < p_b.other_sub;
			}
			return p_a.self_sub < p_b.self_sub;
		}
	};

	JPH::BodyIDVector ids;
	physics_system->GetBodies(ids);
	const JPH::BodyLockInterface &lock_iface = get_lock_iface();

	LocalVector<AreaEntry> areas;
	for (const JPH::BodyID &id : ids) {
		const JPH::Body *body = lock_iface.TryGetBody(id);
		if (body == nullptr) {
			continue;
		}
		JoltObject3D *object = reinterpret_cast<JoltObject3D *>(body->GetUserData());
		if (object == nullptr) {
			continue;
		}
		if (JoltArea3D *area = object->as_area()) {
			areas.push_back({ body->GetStableKey(), id, area });
		}
	}
	areas.sort_custom<AreaEntryLess>();

	for (const AreaEntry &entry : areas) {
		entry.area->exit_all_overlaps();
		entry.area->call_queries();
	}
	contact_listener->clear_area_overlaps();

	JPH::CollideShapeSettings settings;
	settings.mMaxSeparationDistance = 0.0f;
	settings.mActiveEdgeMode = physics_system->GetPhysicsSettings().mCheckActiveEdges ? JPH::EActiveEdgeMode::CollideOnlyWithActive : JPH::EActiveEdgeMode::CollideWithAll;

	LocalVector<Hit> hits;
	for (const AreaEntry &entry : areas) {
		if (!entry.area->is_monitoring()) {
			continue;
		}
		const JPH::Body *body = lock_iface.TryGetBody(entry.id);
		if (body == nullptr) {
			continue;
		}
		JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
		const JPH::IgnoreSingleBodyFilter body_filter(entry.id);
		get_narrow_phase_query().CollideShape(body->GetShape(), JPH::Vec3::sOne(), body->GetCenterOfMassTransform(), settings, JPH::RVec3::sZero(), collector, {}, {}, body_filter);

		hits.clear();
		for (const JPH::CollideShapeResult &result : collector.mHits) {
			const JPH::Body *other = lock_iface.TryGetBody(result.mBodyID2);
			if (other == nullptr) {
				continue;
			}
			hits.push_back({ other->GetStableKey(), result.mSubShapeID2.GetValue(), result.mSubShapeID1.GetValue(), result.mBodyID2, result.mSubShapeID2, result.mSubShapeID1 });
		}
		hits.sort_custom<HitLess>();

		for (uint32_t i = 0; i < hits.size(); i++) {
			const Hit &hit = hits[i];
			if (i > 0 && hits[i - 1].other_id == hit.other_id && hits[i - 1].other_sub == hit.other_sub && hits[i - 1].self_sub == hit.self_sub) {
				continue;
			}
			JoltObject3D *other_object = try_get_object(hit.other_id);
			if (other_object == nullptr) {
				continue;
			}
			const JPH::SubShapeIDPair shape_pair(entry.id, hit.self_sub_id, hit.other_id, hit.other_sub_id);
			if (const JoltArea3D *other_area = other_object->as_area()) {
				if (entry.area->can_monitor(*other_area)) {
					contact_listener->add_area_overlap(shape_pair);
					entry.area->area_shape_entered(hit.other_id, hit.other_sub_id, hit.self_sub_id);
				}
			} else if (const JoltBody3D *other_body = other_object->as_body()) {
				if (entry.area->can_monitor(*other_body)) {
					contact_listener->add_area_overlap(shape_pair);
					entry.area->body_shape_entered(hit.other_id, hit.other_sub_id, hit.self_sub_id);
				}
			}
		}
		entry.area->call_queries();
	}
}

static uint32_t _hash_float_bits(float p_value, uint32_t p_seed) {
	uint32_t bits;
	memcpy(&bits, &p_value, sizeof(bits));
	return hash_murmur3_one_32(bits, p_seed);
}

static uint32_t _hash_vec3(const JPH::Vec3 &p_value, uint32_t p_seed) {
	p_seed = _hash_float_bits(p_value.GetX(), p_seed);
	p_seed = _hash_float_bits(p_value.GetY(), p_seed);
	return _hash_float_bits(p_value.GetZ(), p_seed);
}

static uint32_t _body_state_hash(const JPH::Body &p_body) {
	uint32_t hash = HASH_MURMUR3_SEED;
	hash = hash_murmur3_one_32(uint32_t(p_body.GetMotionType()), hash);
	hash = hash_murmur3_one_32(p_body.IsSensor() ? 1u : 0u, hash);
	hash = hash_murmur3_one_32(p_body.IsInBroadPhase() ? 1u : 0u, hash);
	const JoltObject3D *object = reinterpret_cast<const JoltObject3D *>(p_body.GetUserData());
	if (object != nullptr) {
		hash = hash_murmur3_one_32(object->get_collision_layer(), hash);
		hash = hash_murmur3_one_32(object->get_collision_mask(), hash);
	}
	const JPH::RVec3 position = p_body.GetPosition();
	hash = _hash_float_bits(float(position.GetX()), hash);
	hash = _hash_float_bits(float(position.GetY()), hash);
	hash = _hash_float_bits(float(position.GetZ()), hash);
	const JPH::Quat rotation = p_body.GetRotation();
	hash = _hash_float_bits(rotation.GetX(), hash);
	hash = _hash_float_bits(rotation.GetY(), hash);
	hash = _hash_float_bits(rotation.GetZ(), hash);
	hash = _hash_float_bits(rotation.GetW(), hash);
	if (!p_body.IsStatic()) {
		hash = hash_murmur3_one_32(p_body.IsActive() ? 1u : 0u, hash);
		hash = _hash_vec3(p_body.GetLinearVelocity(), hash);
		hash = _hash_vec3(p_body.GetAngularVelocity(), hash);
	}
	if (p_body.IsDynamic()) {
		const JPH::MotionProperties *motion = p_body.GetMotionPropertiesUnchecked();
		hash = _hash_float_bits(motion->GetGravityFactor(), hash);
		hash = _hash_float_bits(motion->GetInverseMassUnchecked(), hash);
		hash = _hash_float_bits(motion->GetLinearDamping(), hash);
		hash = _hash_float_bits(motion->GetAngularDamping(), hash);
		hash = _hash_vec3(motion->GetAccumulatedForce(), hash);
		hash = _hash_vec3(motion->GetAccumulatedTorque(), hash);
	}
	return hash_fmix32(hash);
}

int64_t JoltSpace3D::get_state_hash(PackedInt64Array *r_bodies) const {
	JPH::BodyIDVector ids;
	physics_system->GetBodies(ids);
	const JPH::BodyLockInterface &lock_iface = get_lock_iface();
	LocalVector<uint32_t> hashes;
	hashes.reserve(ids.size());
	for (const JPH::BodyID &id : ids) {
		const JPH::Body *body = lock_iface.TryGetBody(id);
		if (body == nullptr) {
			continue;
		}
		const uint32_t body_hash = _body_state_hash(*body);
		hashes.push_back(body_hash);
		if (r_bodies != nullptr) {
			const JoltObject3D *object = reinterpret_cast<const JoltObject3D *>(body->GetUserData());
			r_bodies->push_back(object != nullptr ? int64_t(uint64_t(object->get_instance_id())) : 0);
			r_bodies->push_back(int64_t(body_hash));
		}
	}
	hashes.sort();
	uint32_t total = HASH_MURMUR3_SEED;
	for (const uint32_t body_hash : hashes) {
		total = hash_murmur3_one_32(body_hash, total);
	}
	return (int64_t(hashes.size()) << 32) | int64_t(hash_fmix32(total));
}

static String _float_bits(float p_value) {
	uint32_t bits;
	memcpy(&bits, &p_value, sizeof(bits));
	return String::num_uint64(bits, 16).lpad(8, "0");
}

void JoltSpace3D::_step_dump(const char *p_phase) const {
	if (step_dump_first < 0 || steps_since_canonicalize < step_dump_first || steps_since_canonicalize > step_dump_last) {
		return;
	}
	_dump_stable_keys(vformat("%s.%d.c%d.s%d.%s.txt", step_dump_path, OS::get_singleton()->get_process_id(), dump_canonicalize_count, steps_since_canonicalize, p_phase));
}

void JoltSpace3D::_dump_stable_keys(const String &p_path) const {
	Ref<FileAccess> file = FileAccess::open(p_path, FileAccess::WRITE);
	ERR_FAIL_COND(file.is_null());

	JPH::BodyIDVector ids;
	physics_system->GetBodies(ids);
	const JPH::BodyLockInterface &lock_iface = get_lock_iface();

	Vector<String> lines;
	HashMap<uint32_t, String> labels;
	for (const JPH::BodyID &id : ids) {
		const JPH::Body *body = lock_iface.TryGetBody(id);
		if (body == nullptr) {
			continue;
		}

		String name = "?";
		const JoltObject3D *object = reinterpret_cast<const JoltObject3D *>(body->GetUserData());
		if (object != nullptr) {
			if (Node *node = Object::cast_to<Node>(object->get_instance())) {
				name = String(node->get_path());
			}
		}

		const JPH::RVec3 p = body->GetCenterOfMassPosition();
		const JPH::Quat q = body->GetRotation();
		const JPH::AABox b = body->GetShape()->GetLocalBounds();
		String line = name + "\tkey=" + String::num_uint64(body->GetStableKey(), 16).lpad(8, "0");
		line += "\tm=" + itos(int(body->GetMotionType())) + "\ts=" + itos(body->IsSensor() ? 1 : 0) + "\tco=" + String::num_uint64(body->GetCreationOrder());
		line += "\tid=" + itos(int(id.GetIndex())) + "\tai=" + itos(int(body->IsActive() ? body->GetIndexInActiveBodiesInternal() : -1)) + "\ttie=" + String::num_uint64(body->ComputeTieKey(), 16);
		line += "\tp=" + _float_bits(float(p.GetX())) + "," + _float_bits(float(p.GetY())) + "," + _float_bits(float(p.GetZ()));
		line += "\tq=" + _float_bits(q.GetX()) + "," + _float_bits(q.GetY()) + "," + _float_bits(q.GetZ()) + "," + _float_bits(q.GetW());
		if (object != nullptr && object->as_body() != nullptr && body->IsDynamic()) {
			line += "\t" + object->as_body()->get_debug_integration_state();
		}
		if (body->GetMotionPropertiesUnchecked() != nullptr) {
			const JPH::Vec3 v = body->GetLinearVelocity();
			const JPH::Vec3 w = body->GetAngularVelocity();
			line += "\tv=" + _float_bits(v.GetX()) + "," + _float_bits(v.GetY()) + "," + _float_bits(v.GetZ());
			line += "\tw=" + _float_bits(w.GetX()) + "," + _float_bits(w.GetY()) + "," + _float_bits(w.GetZ());
		}
		line += "\tb=" + _float_bits(b.mMin.GetX()) + "," + _float_bits(b.mMin.GetY()) + "," + _float_bits(b.mMin.GetZ()) + "," + _float_bits(b.mMax.GetX()) + "," + _float_bits(b.mMax.GetY()) + "," + _float_bits(b.mMax.GetZ());
		const JPH::Vec3 com = body->GetShape()->GetCenterOfMass();
		line += "\tst=" + itos(int(body->GetShape()->GetSubType())) + "\tcom=" + _float_bits(com.GetX()) + "," + _float_bits(com.GetY()) + "," + _float_bits(com.GetZ());
		line += "\tfr=" + _float_bits(body->GetFriction()) + "\tre=" + _float_bits(body->GetRestitution());
		const JPH::RVec3 origin = body->GetPosition();
		line += "\top=" + _float_bits(float(origin.GetX())) + "," + _float_bits(float(origin.GetY())) + "," + _float_bits(float(origin.GetZ()));
		if (object != nullptr && object->as_shaped() != nullptr) {
			const Vector3 shaped_scale = object->as_shaped()->get_scale();
			line += "\tsc=" + _float_bits(shaped_scale.x) + "," + _float_bits(shaped_scale.y) + "," + _float_bits(shaped_scale.z);
		}
		if (body->IsDynamic()) {
			const JPH::MotionProperties *mp = body->GetMotionPropertiesUnchecked();
			const JPH::Vec3 ii = mp->GetInverseInertiaDiagonal();
			const JPH::Quat ir = mp->GetInertiaRotation();
			line += "\tim=" + _float_bits(mp->GetInverseMassUnchecked());
			line += "\tii=" + _float_bits(ii.GetX()) + "," + _float_bits(ii.GetY()) + "," + _float_bits(ii.GetZ());
			line += "\tir=" + _float_bits(ir.GetX()) + "," + _float_bits(ir.GetY()) + "," + _float_bits(ir.GetZ()) + "," + _float_bits(ir.GetW());
			line += "\tdof=" + itos(int(mp->GetAllowedDOFs())) + "\tmq=" + itos(int(mp->GetMotionQuality()));
			line += "\tjd=" + _float_bits(mp->GetLinearDamping()) + "," + _float_bits(mp->GetAngularDamping()) + "\tgf=" + _float_bits(mp->GetGravityFactor()) + "\tmlv=" + _float_bits(mp->GetMaxLinearVelocity());
			const JPH::Vec3 af = mp->GetAccumulatedForce();
			const JPH::Vec3 at = mp->GetAccumulatedTorque();
			line += "\taf=" + _float_bits(af.GetX()) + "," + _float_bits(af.GetY()) + "," + _float_bits(af.GetZ());
			line += "\tat=" + _float_bits(at.GetX()) + "," + _float_bits(at.GetY()) + "," + _float_bits(at.GetZ());
			line += "\tsteps=" + itos(mp->GetNumVelocityStepsOverride()) + "," + itos(mp->GetNumPositionStepsOverride());
		}
		lines.push_back(line);
		labels[id.GetIndexAndSequenceNumber()] = String::num_uint64(body->GetStableKey(), 16).lpad(8, "0") + "|" + name;

		const JPH::Shape *inner = body->GetShape();
		String wrappers;
		while (inner->GetType() == JPH::EShapeType::Decorated) {
			wrappers += itos(int(inner->GetSubType())) + ">";
			if (inner->GetSubType() == JPH::EShapeSubType::OffsetCenterOfMass) {
				const JPH::Vec3 off = static_cast<const JPH::OffsetCenterOfMassShape *>(inner)->GetOffset();
				wrappers += "off(" + _float_bits(off.GetX()) + "," + _float_bits(off.GetY()) + "," + _float_bits(off.GetZ()) + ")>";
			}
			inner = static_cast<const JPH::DecoratedShape *>(inner)->GetInnerShape();
		}
		if (inner->GetType() == JPH::EShapeType::Compound) {
			const JPH::CompoundShape *compound = static_cast<const JPH::CompoundShape *>(inner);
			const JPH::Vec3 ccom = compound->GetCenterOfMass();
			lines.push_back("~compound\t" + String::num_uint64(body->GetStableKey(), 16).lpad(8, "0") + "\t" + wrappers + itos(int(inner->GetSubType())) + "\tn=" + itos(compound->GetNumSubShapes()) + "\tcom=" + _float_bits(ccom.GetX()) + "," + _float_bits(ccom.GetY()) + "," + _float_bits(ccom.GetZ()) + "\t" + name);
			for (JPH::uint i = 0; i < compound->GetNumSubShapes(); ++i) {
				const JPH::CompoundShape::SubShape &sub = compound->GetSubShape(i);
				const JPH::Vec3 sp = sub.GetPositionCOM();
				const JPH::Quat sr = sub.GetRotation();
				const JPH::AABox sb = sub.mShape->GetLocalBounds();
				String sub_line = "~sub\t" + String::num_uint64(body->GetStableKey(), 16).lpad(8, "0") + "\t" + itos(i) + "\tud=" + itos(sub.mUserData) + "\tst=" + itos(int(sub.mShape->GetSubType()));
				sub_line += "\tp=" + _float_bits(sp.GetX()) + "," + _float_bits(sp.GetY()) + "," + _float_bits(sp.GetZ());
				sub_line += "\tq=" + _float_bits(sr.GetX()) + "," + _float_bits(sr.GetY()) + "," + _float_bits(sr.GetZ()) + "," + _float_bits(sr.GetW());
				sub_line += "\tb=" + _float_bits(sb.mMin.GetX()) + "," + _float_bits(sb.mMin.GetY()) + "," + _float_bits(sb.mMin.GetZ()) + "," + _float_bits(sb.mMax.GetX()) + "," + _float_bits(sb.mMax.GetY()) + "," + _float_bits(sb.mMax.GetZ());
				lines.push_back(sub_line);
			}
		}
	}

	physics_system->ForEachCachedContactPoint([&](const JPH::SubShapeIDPair &p_key, const JPH::Float3 &p_normal, JPH::uint p_index, const JPH::Float3 &p_position1, const JPH::Float3 &p_position2, float p_lambda, float p_friction1, float p_friction2) {
		const String *label1 = labels.getptr(p_key.GetBody1ID().GetIndexAndSequenceNumber());
		const String *label2 = labels.getptr(p_key.GetBody2ID().GetIndexAndSequenceNumber());
		const String l1 = label1 != nullptr ? *label1 : String("?|?");
		const String l2 = label2 != nullptr ? *label2 : String("?|?");
		String line = "~contact\t" + l1.get_slicec('|', 0) + "\t" + l2.get_slicec('|', 0);
		line += "\tsub=" + String::num_uint64(p_key.GetSubShapeID1().GetValue(), 16) + "," + String::num_uint64(p_key.GetSubShapeID2().GetValue(), 16) + "\ti=" + itos(p_index);
		line += "\tn=" + _float_bits(p_normal.x) + "," + _float_bits(p_normal.y) + "," + _float_bits(p_normal.z);
		line += "\tp1=" + _float_bits(p_position1.x) + "," + _float_bits(p_position1.y) + "," + _float_bits(p_position1.z);
		line += "\tp2=" + _float_bits(p_position2.x) + "," + _float_bits(p_position2.y) + "," + _float_bits(p_position2.z);
		line += "\tl=" + _float_bits(p_lambda) + "," + _float_bits(p_friction1) + "," + _float_bits(p_friction2);
		line += "\t" + l1.get_slicec('|', 1) + "\t" + l2.get_slicec('|', 1);
		lines.push_back(line);
	});

	lines.sort();
	for (const String &line : lines) {
		file->store_line(line);
	}
}

void JoltSpace3D::flush_pending_objects() {
	if (pending_objects_sleeping.is_empty() && pending_objects_awake.is_empty()) {
		return;
	}

	// We only care about locking within this method, because it's called when performing queries, which aren't covered by `PhysicsServer3DWrapMT`.
	MutexLock pending_objects_lock(pending_objects_mutex);

	JPH::BodyInterface &body_iface = get_body_iface();

	if (!pending_objects_sleeping.is_empty()) {
		JPH::BodyInterface::AddState add_state = body_iface.AddBodiesPrepare(pending_objects_sleeping.ptr(), pending_objects_sleeping.size());
		body_iface.AddBodiesFinalize(pending_objects_sleeping.ptr(), pending_objects_sleeping.size(), add_state, JPH::EActivation::DontActivate);
		pending_objects_sleeping.reset();
	}

	if (!pending_objects_awake.is_empty()) {
		JPH::BodyInterface::AddState add_state = body_iface.AddBodiesPrepare(pending_objects_awake.ptr(), pending_objects_awake.size());
		body_iface.AddBodiesFinalize(pending_objects_awake.ptr(), pending_objects_awake.size(), add_state, JPH::EActivation::Activate);
		pending_objects_awake.reset();
	}
}

void JoltSpace3D::set_is_object_sleeping(const JPH::BodyID &p_jolt_id, bool p_enable) {
	if (p_enable) {
		if (pending_objects_awake.erase_unordered(p_jolt_id)) {
			pending_objects_sleeping.push_back(p_jolt_id);
		} else if (pending_objects_sleeping.has(p_jolt_id)) {
			// Do nothing.
		} else {
			get_body_iface().DeactivateBody(p_jolt_id);
		}
	} else {
		if (pending_objects_sleeping.erase_unordered(p_jolt_id)) {
			pending_objects_awake.push_back(p_jolt_id);
		} else if (pending_objects_awake.has(p_jolt_id)) {
			// Do nothing.
		} else {
			get_body_iface().ActivateBody(p_jolt_id);
		}
	}
}

void JoltSpace3D::enqueue_call_queries(SelfList<JoltBody3D> *p_body) {
	// This method will be called from the body activation listener on multiple threads during the simulation step.
	MutexLock body_call_queries_lock(body_call_queries_mutex);

	if (!p_body->in_list()) {
		body_call_queries_list.add(p_body);
	}
}

void JoltSpace3D::enqueue_call_queries(SelfList<JoltArea3D> *p_area) {
	if (!p_area->in_list()) {
		area_call_queries_list.add(p_area);
	}
}

void JoltSpace3D::dequeue_call_queries(SelfList<JoltBody3D> *p_body) {
	if (p_body->in_list()) {
		body_call_queries_list.remove(p_body);
	}
}

void JoltSpace3D::dequeue_call_queries(SelfList<JoltArea3D> *p_area) {
	if (p_area->in_list()) {
		area_call_queries_list.remove(p_area);
	}
}

void JoltSpace3D::enqueue_shapes_changed(SelfList<JoltShapedObject3D> *p_object) {
	if (!p_object->in_list()) {
		shapes_changed_list.add(p_object);
	}
}

void JoltSpace3D::dequeue_shapes_changed(SelfList<JoltShapedObject3D> *p_object) {
	if (p_object->in_list()) {
		shapes_changed_list.remove(p_object);
	}
}

void JoltSpace3D::enqueue_needs_optimization(SelfList<JoltShapedObject3D> *p_object) {
	if (!p_object->in_list()) {
		needs_optimization_list.add(p_object);
	}
}

void JoltSpace3D::dequeue_needs_optimization(SelfList<JoltShapedObject3D> *p_object) {
	if (p_object->in_list()) {
		needs_optimization_list.remove(p_object);
	}
}

void JoltSpace3D::add_joint(JPH::Constraint *p_jolt_ref) {
	physics_system->AddConstraint(p_jolt_ref);
}

void JoltSpace3D::add_joint(JoltJoint3D *p_joint) {
	add_joint(p_joint->get_jolt_ref());
}

void JoltSpace3D::remove_joint(JPH::Constraint *p_jolt_ref) {
	physics_system->RemoveConstraint(p_jolt_ref);
}

void JoltSpace3D::remove_joint(JoltJoint3D *p_joint) {
	remove_joint(p_joint->get_jolt_ref());
}

#ifdef DEBUG_ENABLED

void JoltSpace3D::dump_debug_snapshot(const String &p_dir) {
	const Dictionary datetime = Time::get_singleton()->get_datetime_dict_from_system();
	const String datetime_str = vformat("%04d-%02d-%02d_%02d-%02d-%02d", datetime["year"], datetime["month"], datetime["day"], datetime["hour"], datetime["minute"], datetime["second"]);
	const String path = p_dir + vformat("/jolt_snapshot_%s_%d.bin", datetime_str, rid.get_id());

	Ref<FileAccess> file_access = FileAccess::open(path, FileAccess::ModeFlags::WRITE);
	ERR_FAIL_COND_MSG(file_access.is_null(), vformat("Failed to open '%s' for writing when saving snapshot of physics space with RID '%d'.", path, rid.get_id()));

	JPH::PhysicsScene physics_scene;
	physics_scene.FromPhysicsSystem(physics_system);

	for (JPH::BodyCreationSettings &settings : physics_scene.GetBodies()) {
		const JoltObject3D *object = reinterpret_cast<const JoltObject3D *>(settings.mUserData);

		if (const JoltBody3D *body = object->as_body()) {
			// Since we do our own integration of gravity and damping, while leaving Jolt's own values at zero, we need to transfer over the correct values.
			settings.mGravityFactor = body->get_gravity_scale();
			settings.mLinearDamping = body->get_total_linear_damp();
			settings.mAngularDamping = body->get_total_angular_damp();
		}

		settings.SetShape(JoltShape3D::without_custom_shapes(settings.GetShape()));
	}

	JoltStreamOutputWrapper output_stream(file_access);
	physics_scene.SaveBinaryState(output_stream, true, false);

	ERR_FAIL_COND_MSG(file_access->get_error() != OK, vformat("Writing snapshot of physics space with RID '%d' to '%s' failed with error '%s'.", rid.get_id(), path, VariantUtilityFunctions::error_string(file_access->get_error())));

	print_line(vformat("Snapshot of physics space with RID '%d' saved to '%s'.", rid.get_id(), path));
}

const PackedVector3Array &JoltSpace3D::get_debug_contacts() const {
	return contact_listener->get_debug_contacts();
}

int JoltSpace3D::get_debug_contact_count() const {
	return contact_listener->get_debug_contact_count();
}

int JoltSpace3D::get_max_debug_contacts() const {
	return contact_listener->get_max_debug_contacts();
}

void JoltSpace3D::set_max_debug_contacts(int p_count) {
	contact_listener->set_max_debug_contacts(p_count);
}

#endif
