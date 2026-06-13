#import "Basic";
#import "Math";

#import "tbx/math";
#import "tbx/turns";

KINEMATIC_CHARACTER_MAX_ITERATIONS :: 8;
KINEMATIC_CHARACTER_COLLISION_TOLERANCE :: 0.01;
KINEMATIC_CHARACTER_PENETRATION_RECOVERY_DISTANCE :: 0.001;
KINEMATIC_CHARACTER_MAX_PENETRATION_RECOVERY_PER_ITERATION :: 0.025;
KINEMATIC_CHARACTER_MAX_SOLVE_STEP_DISTANCE :: 0.02;
KINEMATIC_CHARACTER_MAX_CONTACTS :: 64;
KINEMATIC_CHARACTER_STEP_UP_HEIGHT :: 0.2;
KINEMATIC_CHARACTER_STEP_DOWN_EXTRA :: 0.05;
KINEMATIC_CHARACTER_MIN_STEP_FORWARD :: 0.02;
KINEMATIC_CHARACTER_CONFLICTING_CONTACT_MIN_PENETRATION :: KINEMATIC_CHARACTER_COLLISION_TOLERANCE * 1.25;
KINEMATIC_CHARACTER_DUPLICATE_CONTACT_POSITION_EPSILON :: 0.0001;
KINEMATIC_CHARACTER_DUPLICATE_CONTACT_NORMAL_DOT :: 0.999;

Kinematic_Character_Ground_State :: enum {
    IN_AIR;
    NOT_SUPPORTED;
    ON_STEEP_GROUND;
    ON_GROUND;
}

Kinematic_Character_Contact :: struct {
    position: Vector3;
    contact_normal: Vector3;
    surface_normal: Vector3;
    distance: float;
    had_collision := false;
}

Kinematic_Character_Result :: struct {
    new_position: Vector3;
    new_displacement: Vector3;
    ground_normal: Vector3;
    ground_state := Kinematic_Character_Ground_State.IN_AIR;
    on_ground := false;
    collision_occurred := false;
}

closest_point_on_triangle_for_kinematic_character :: (p: Vector3, tri: Triangle) -> Vector3 {
    ab := tri.p1 - tri.p0;
    ac := tri.p2 - tri.p0;
    ap := p - tri.p0;

    d1 := dot(ab, ap);
    d2 := dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 return tri.p0;

    bp := p - tri.p1;
    d3 := dot(ab, bp);
    d4 := dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 return tri.p1;

    vc := d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        v := d1 / (d1 - d3);
        return tri.p0 + v * ab;
    }

    cp := p - tri.p2;
    d5 := dot(ab, cp);
    d6 := dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 return tri.p2;

    vb := d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        w := d2 / (d2 - d6);
        return tri.p0 + w * ac;
    }

    va := d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        w := (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return tri.p1 + w * (tri.p2 - tri.p1);
    }

    denom := 1.0 / (va + vb + vc);
    v := vb * denom;
    w := vc * denom;
    return tri.p0 + ab * v + ac * w;
}

append_kinematic_character_contact :: (
    contacts: *[..]Kinematic_Character_Contact,
    e_position: Vector3,
    e_tri: Triangle,
    world_tri: Triangle,
    ellipsoid_radius: Vector3,
    tolerance: float
) {
    if contacts.count >= KINEMATIC_CHARACTER_MAX_CONTACTS return;

    closest := closest_point_on_triangle_for_kinematic_character(e_position, e_tri);
    to_center := e_position - closest;
    distance_to_surface := length(to_center);
    distance := distance_to_surface - 1.0;
    if distance > tolerance return;

    contact_normal_espace: Vector3;
    if distance_to_surface > EPSILON_ {
        contact_normal_espace = to_center / distance_to_surface;
    } else {
        contact_normal_espace = plane_from_triangle(e_tri).normal;
    }

    contact_normal_world := ellipsoid_normal_to_world(contact_normal_espace, ellipsoid_radius);
    surface_normal_world := plane_from_triangle(world_tri).normal;

    if dot(contact_normal_world, surface_normal_world) < -0.0174524 {
        surface_normal_world = -surface_normal_world;
    }

    if dot(contact_normal_world, Y3) > dot(surface_normal_world, Y3) {
        surface_normal_world = contact_normal_world;
    }

    contact_position_world := ellipsoid_to_world(closest, ellipsoid_radius);
    array_add(contacts, .{
        position = contact_position_world,
        contact_normal = contact_normal_world,
        surface_normal = surface_normal_world,
        distance = distance,
        had_collision = true
    });
}

collect_kinematic_character_contacts :: (
    positioned_colliders: Collision_Geometries,
    position: Vector3,
    ellipsoid_radius: Vector3,
    tolerance: float,
    contacts: *[..]Kinematic_Character_Contact
) {
    contacts.count = 0;

    body_aabb := make_ellipsoid_movement_aabb(position, ZERO3, ZERO3, ellipsoid_radius + Vector3.{tolerance, tolerance, tolerance});
    e_position := world_to_ellipsoid(position, ellipsoid_radius);

    for 0..positioned_colliders.aabbs.count-1 {
        collider := Collision_Geometry.{positioned_colliders.indexed_triangle_positions[it], positioned_colliders.aabbs[it]};
        if aabbs_are_disjoint(collider.aabb, body_aabb) continue;

        indices := collider.indexed_triangle_positions.indices;
        positions := collider.indexed_triangle_positions.positions;
        triangle_count := indices.count / 3;

        for tri_index: 0..triangle_count-1 {
            world_tri: Triangle;
            world_tri.p0 = positions[indices[tri_index * 3 + 0]];
            world_tri.p1 = positions[indices[tri_index * 3 + 1]];
            world_tri.p2 = positions[indices[tri_index * 3 + 2]];

            e_tri: Triangle;
            e_tri.p0 = world_to_ellipsoid(world_tri.p0, ellipsoid_radius);
            e_tri.p1 = world_to_ellipsoid(world_tri.p1, ellipsoid_radius);
            e_tri.p2 = world_to_ellipsoid(world_tri.p2, ellipsoid_radius);

            append_kinematic_character_contact(contacts, e_position, e_tri, world_tri, ellipsoid_radius, tolerance);
            if contacts.count >= KINEMATIC_CHARACTER_MAX_CONTACTS return;
        }
    }
}

kinematic_character_recovery_normal :: (contact: Kinematic_Character_Contact) -> Vector3 {
    if is_walkable_ground_normal(contact.surface_normal) && is_walkable_ground_normal(contact.contact_normal) {
        return contact.surface_normal;
    }

    return contact.contact_normal;
}

depenetrate_kinematic_character_position :: (
    positioned_colliders: Collision_Geometries,
    position: Vector3,
    ellipsoid_radius: Vector3,
    contacts: *[..]Kinematic_Character_Contact
) -> Vector3, bool {
    result := position;
    collided := false;

    for 0..KINEMATIC_CHARACTER_MAX_ITERATIONS-1 {
        collect_kinematic_character_contacts(positioned_colliders, result, ellipsoid_radius, KINEMATIC_CHARACTER_COLLISION_TOLERANCE, contacts);
        cleanup_kinematic_character_contacts(contacts);

        largest_push := 0.0;
        chosen_push := 0.0;
        chosen_push_normal := ZERO3;
        chosen_distance := 0.0;
        penetration_count := 0;
        for contacts.* {
            if it.distance >= 0.0 continue;

            push := -it.distance + KINEMATIC_CHARACTER_PENETRATION_RECOVERY_DISTANCE;
            push_normal := kinematic_character_recovery_normal(it);
            largest_push = max(largest_push, push);
            penetration_count += 1;

            if push > chosen_push {
                chosen_push = push;
                chosen_push_normal = push_normal;
                chosen_distance = it.distance;
            }
        }

        applied_push := min(chosen_push, KINEMATIC_CHARACTER_MAX_PENETRATION_RECOVERY_PER_ITERATION);
        if applied_push > 0.0 {
            result += chosen_push_normal * applied_push;
            collided = true;
        }


        if chosen_push <= 0.0001 break;
    }

    return result, collided;
}

solve_kinematic_character_position :: (
    positioned_colliders: Collision_Geometries,
    position: Vector3,
    desired_displacement: Vector3,
    ellipsoid_radius: Vector3,
    contacts: *[..]Kinematic_Character_Contact
) -> Vector3, bool {
    displacement_length := length(desired_displacement);
    if displacement_length <= EPSILON_ {
        result, collided := depenetrate_kinematic_character_position(positioned_colliders, position, ellipsoid_radius, contacts);
        return result, collided;
    }

    step_count := cast(int) ceil(displacement_length / KINEMATIC_CHARACTER_MAX_SOLVE_STEP_DISTANCE);
    if step_count < 1 step_count = 1;

    step_displacement := desired_displacement / cast(float) step_count;
    result := position;
    collided := false;

    for step_index: 0..step_count-1 {
        result += step_displacement;
        depenetrated_result, step_collided := depenetrate_kinematic_character_position(positioned_colliders, result, ellipsoid_radius, contacts);
        result = depenetrated_result;
        if step_collided collided = true;
    }

    if step_count > 1 {
    }

    return result, collided;
}

kinematic_character_contacts_are_duplicates :: (a: Kinematic_Character_Contact, b: Kinematic_Character_Contact) -> bool {
    if length_squared(a.position - b.position) > KINEMATIC_CHARACTER_DUPLICATE_CONTACT_POSITION_EPSILON * KINEMATIC_CHARACTER_DUPLICATE_CONTACT_POSITION_EPSILON return false;
    if dot(a.contact_normal, b.contact_normal) < KINEMATIC_CHARACTER_DUPLICATE_CONTACT_NORMAL_DOT return false;
    return true;
}

merge_kinematic_character_contacts :: (a: *Kinematic_Character_Contact, b: Kinematic_Character_Contact) {
    if b.distance < a.distance {
        a.distance = b.distance;
        a.position = b.position;
        a.contact_normal = b.contact_normal;
    }

    if dot(b.surface_normal, Y3) > dot(a.surface_normal, Y3) {
        a.surface_normal = b.surface_normal;
    }
}

cleanup_kinematic_character_contacts :: (contacts: *[..]Kinematic_Character_Contact) {
    duplicate_removed := 0;
    conflicting_removed := 0;

    i := 0;
    while i < contacts.count {
        j := i + 1;
        while j < contacts.count {
            if kinematic_character_contacts_are_duplicates(contacts.*[i], contacts.*[j]) {
                merge_kinematic_character_contacts(*contacts.*[i], contacts.*[j]);
                array_ordered_remove_by_index(contacts, j);
                duplicate_removed += 1;
                continue;
            }

            j += 1;
        }

        i += 1;
    }

    i = 0;
    while i < contacts.count {
        if contacts.*[i].distance <= -KINEMATIC_CHARACTER_CONFLICTING_CONTACT_MIN_PENETRATION {
            j := i + 1;
            removed_i := false;
            while j < contacts.count {
                if contacts.*[j].distance <= -KINEMATIC_CHARACTER_CONFLICTING_CONTACT_MIN_PENETRATION &&
                   dot(contacts.*[i].contact_normal, contacts.*[j].contact_normal) < 0.0 {
                    if contacts.*[i].distance < contacts.*[j].distance {
                        array_ordered_remove_by_index(contacts, j);
                    } else {
                        array_ordered_remove_by_index(contacts, i);
                        removed_i = true;
                    }

                    conflicting_removed += 1;
                    if removed_i break;
                    continue;
                }

                j += 1;
            }

            if removed_i continue;
        }

        i += 1;
    }

    if duplicate_removed > 0 || conflicting_removed > 0 {
    }
}

cancel_velocity_towards_kinematic_steep_slopes :: (velocity: *Vector3, contacts: []Kinematic_Character_Contact) {
    for contacts {
        if !it.had_collision continue;
        if is_walkable_ground_normal(it.surface_normal) continue;

        normal := it.contact_normal;
        normal -= dot(normal, Y3) * Y3;
        len_sq := length_squared(normal);
        if len_sq <= EPSILON_ * EPSILON_ continue;

        into_slope := dot(normal, velocity.*);
        if into_slope < 0.0 {
            before := velocity.*;
            velocity.* -= (into_slope / len_sq) * normal;
        }
    }
}

clip_velocity_against_kinematic_contacts :: (velocity: *Vector3, contacts: []Kinematic_Character_Contact) {
    previous_normals: [..] Vector3;
    previous_normals.allocator = temp;

    for contacts {
        if !it.had_collision continue;
        if is_walkable_ground_normal(it.surface_normal) continue;

        plane_normal := it.contact_normal;
        into_contact := dot(velocity.*, plane_normal);
        if into_contact < 0.0 {
            before := velocity.*;
            new_velocity := velocity.* - plane_normal * into_contact;
            crease_normal: Vector3;
            found_crease := false;
            highest_penetration := 0.0;

            for previous_normals {
                penetration := -dot(new_velocity, it);
                if penetration > highest_penetration {
                    normal_dot := dot(it, plane_normal);
                    if normal_dot < 0.984 && normal_dot > -0.984 {
                        highest_penetration = penetration;
                        crease_normal = it;
                        found_crease = true;
                    }
                }
            }

            if found_crease {
                slide_dir := cross(plane_normal, crease_normal);
                slide_len := length(slide_dir);
                if slide_len > EPSILON_ {
                    slide_dir /= slide_len;
                    new_velocity = dot(new_velocity, slide_dir) * slide_dir;
                }
            }

            velocity.* = new_velocity;
        }

        should_add := true;
        for previous_normals {
            normal_dot := dot(it, plane_normal);
            if normal_dot > 0.984 || normal_dot < -0.984 {
                should_add = false;
                break;
            }
        }

        if should_add && previous_normals.count < KINEMATIC_CHARACTER_MAX_CONTACTS {
            array_add(*previous_normals, plane_normal);
        }
    }
}

align_kinematic_velocity_to_walkable_ground :: (velocity: *Vector3, ground_normal: Vector3) {
    horizontal := Vector3.{velocity.x, 0, velocity.z};
    horizontal_speed := length(horizontal);
    if horizontal_speed <= EPSILON_ return;
    if abs(ground_normal.y) <= EPSILON_ return;

    ramp_aligned := horizontal;
    ramp_aligned.y = -(ground_normal.x * horizontal.x + ground_normal.z * horizontal.z) / ground_normal.y;

    before := velocity.*;
    velocity.* = ramp_aligned;
}

kinematic_character_has_steep_contact_in_direction :: (contacts: []Kinematic_Character_Contact, direction: Vector3) -> bool {
    for contacts {
        if !it.had_collision continue;
        if is_walkable_ground_normal(it.surface_normal) continue;
        if dot(it.surface_normal, direction) < 0.0 return true;
    }

    return false;
}

try_kinematic_character_walk_stairs :: (
    positioned_colliders: Collision_Geometries,
    ellipsoid_radius: Vector3,
    start_position: Vector3,
    normal_position: Vector3,
    desired_displacement: Vector3,
    active_contacts: []Kinematic_Character_Contact
) -> Vector3, bool {
    desired_horizontal := Vector3.{desired_displacement.x, 0, desired_displacement.z};
    desired_horizontal_len := length(desired_horizontal);
    if desired_horizontal_len <= EPSILON_ return normal_position, false;

    forward_dir := desired_horizontal / desired_horizontal_len;
    achieved_horizontal := Vector3.{normal_position.x - start_position.x, 0, normal_position.z - start_position.z};
    achieved_forward_len := max(0.0, dot(achieved_horizontal, forward_dir));
    if achieved_forward_len + 0.0001 >= desired_horizontal_len return normal_position, false;
    if !kinematic_character_has_steep_contact_in_direction(active_contacts, forward_dir) return normal_position, false;

    step_forward_len := max(KINEMATIC_CHARACTER_MIN_STEP_FORWARD, desired_horizontal_len - achieved_forward_len);
    step_up := Y3 * KINEMATIC_CHARACTER_STEP_UP_HEIGHT;
    step_forward := forward_dir * step_forward_len;
    step_down := -Y3 * (KINEMATIC_CHARACTER_STEP_UP_HEIGHT + KINEMATIC_CHARACTER_STEP_DOWN_EXTRA);

    contacts: [..] Kinematic_Character_Contact;
    contacts.allocator = temp;

    up_position, up_collided := solve_kinematic_character_position(positioned_colliders, start_position, step_up, ellipsoid_radius, *contacts);
    up_delta_y := up_position.y - start_position.y;
    if up_delta_y <= KINEMATIC_CHARACTER_COLLISION_TOLERANCE {
        return normal_position, false;
    }

    forward_position, forward_collided := solve_kinematic_character_position(positioned_colliders, up_position, step_forward, ellipsoid_radius, *contacts);
    forward_horizontal := Vector3.{forward_position.x - up_position.x, 0, forward_position.z - up_position.z};
    forward_progress := dot(forward_horizontal, forward_dir);
    if forward_progress <= 0.0001 {
        return normal_position, false;
    }

    down_position, down_collided := solve_kinematic_character_position(positioned_colliders, forward_position, step_down, ellipsoid_radius, *contacts);
    stepped_delta_y := down_position.y - start_position.y;
    if stepped_delta_y > KINEMATIC_CHARACTER_STEP_UP_HEIGHT + KINEMATIC_CHARACTER_COLLISION_TOLERANCE {
        return normal_position, false;
    }

    collect_kinematic_character_contacts(positioned_colliders, down_position, ellipsoid_radius, KINEMATIC_CHARACTER_COLLISION_TOLERANCE, *contacts);
    ground_state, ground_normal := classify_kinematic_character_ground(ZERO3, contacts);
    if ground_state != .ON_GROUND {
        return normal_position, false;
    }

    stepped_horizontal := Vector3.{down_position.x - start_position.x, 0, down_position.z - start_position.z};
    stepped_forward_len := dot(stepped_horizontal, forward_dir);
    if stepped_forward_len <= achieved_forward_len + 0.0001 {
        return normal_position, false;
    }


    return down_position, true;
}

classify_kinematic_character_ground :: (velocity: Vector3, contacts: []Kinematic_Character_Contact) -> Kinematic_Character_Ground_State, Vector3 {
    supported_count := 0;
    sliding_count := 0;
    best_contact: *Kinematic_Character_Contact = null;
    best_up := -FLOAT32_MAX;
    avg_normal := ZERO3;
    avg_count := 0;

    for * contacts {
        if !it.had_collision continue;
        if it.distance > KINEMATIC_CHARACTER_COLLISION_TOLERANCE continue;
        moving_away := dot(it.surface_normal, velocity);
        if moving_away > 1.0e-4 {
            continue;
        }

        cos_angle := dot(it.surface_normal, Y3);
        if cos_angle > best_up {
            best_up = cos_angle;
            best_contact = it;
        }

        if is_walkable_ground_normal(it.surface_normal) {
            supported_count += 1;
        } else {
            sliding_count += 1;
        }

        if cos_angle >= 0.08 {
            avg_normal += it.surface_normal;
            avg_count += 1;
        }
    }

    ground_normal := Y3;
    if avg_count > 0 {
        ground_normal = normalize(avg_normal);
    } else if best_contact != null {
        ground_normal = best_contact.surface_normal;
    }

    result: Kinematic_Character_Ground_State;
    if supported_count > 0 {
        result = .ON_GROUND;
    } else if sliding_count > 0 {
        result = .ON_STEEP_GROUND;
    } else if best_contact != null {
        result = .NOT_SUPPORTED;
    } else {
        result = .IN_AIR;
        ground_normal = Y3;
    }


    return result, ground_normal;
}

append_kinematic_character_contacts_for_support :: (dst: *[..]Kinematic_Character_Contact, src: []Kinematic_Character_Contact) {
    for src {
        if !it.had_collision continue;
        if it.distance > KINEMATIC_CHARACTER_COLLISION_TOLERANCE continue;
        if dst.count >= KINEMATIC_CHARACTER_MAX_CONTACTS return;
        array_add(dst, it);
    }
}

resolve_collisions_against_kinematic_character :: (
    positioned_colliders: Collision_Geometries,
    ellipsoid_radius: Vector3,
    body_position: *Vector3,
    velocity: *Vector3,
    dt: float,
    on_ground: *bool,
    gravity_displacement: Vector3 = .{0, 0, 0}
) -> Kinematic_Character_Result {
    desired_displacement := velocity.* * dt + gravity_displacement;
    start_position := body_position.*;
    start_velocity := velocity.*;
    was_on_ground := on_ground.*;


    contacts: [..] Kinematic_Character_Contact;
    contacts.allocator = temp;
    active_contacts: [..] Kinematic_Character_Contact;
    active_contacts.allocator = temp;

    new_position, collided := solve_kinematic_character_position(
        positioned_colliders,
        body_position.*,
        desired_displacement,
        ellipsoid_radius,
        *contacts
    );
    collect_kinematic_character_contacts(positioned_colliders, new_position, ellipsoid_radius, KINEMATIC_CHARACTER_COLLISION_TOLERANCE, *contacts);
    append_kinematic_character_contacts_for_support(*active_contacts, contacts);
    cleanup_kinematic_character_contacts(*active_contacts);

    if was_on_ground {
        stepped_position, stepped := try_kinematic_character_walk_stairs(
            positioned_colliders,
            ellipsoid_radius,
            start_position,
            new_position,
            desired_displacement,
            active_contacts
        );

        if stepped {
            new_position = stepped_position;
            active_contacts.count = 0;
            collect_kinematic_character_contacts(positioned_colliders, new_position, ellipsoid_radius, KINEMATIC_CHARACTER_COLLISION_TOLERANCE, *contacts);
            append_kinematic_character_contacts_for_support(*active_contacts, contacts);
            cleanup_kinematic_character_contacts(*active_contacts);
        }
    }

    cancel_velocity_towards_kinematic_steep_slopes(velocity, active_contacts);
    clip_velocity_against_kinematic_contacts(velocity, active_contacts);

    ground_state, ground_normal := classify_kinematic_character_ground(velocity.*, active_contacts);
    on_ground.* = ground_state == .ON_GROUND;
    if on_ground.* {
        align_kinematic_velocity_to_walkable_ground(velocity, ground_normal);
    }

    result: Kinematic_Character_Result;
    result.new_position = new_position;
    result.new_displacement = new_position - body_position.*;
    result.ground_state = ground_state;
    result.ground_normal = ground_normal;
    result.on_ground = on_ground.*;
    result.collision_occurred = collided || contacts.count > 0;

    body_position.* = new_position;
    return result;
}
