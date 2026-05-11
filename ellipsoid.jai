/*
implementation of kasper fauerby's "improved collision detection and response"
http://www.peroxide.dk/papers/collision/collision.pdf

The character is represented as an ellipsoid in world space. By dividing points
and displacements by the ellipsoid radius, the problem becomes a swept unit
sphere test in ellipsoid space. The response below follows the paper's recursive
collide-and-slide step: move near the closest hit, build the sphere tangent
sliding plane, project the destination to that plane, and recurse.
*/

#import "Basic";
#import "Math";

#import "tbx/math";

Recursive_Collision_Data :: struct {
    // ellipsoid radius (in world space)
    ellipsoid_radius: Vector3;

    // information about the move being requested (in ellipsoid space)
    e_displacement: Vector3;
    e_normalized_displacement: Vector3;
    e_base_point: Vector3;

    // hit information
    found_collision: bool;
    parametric_distance_along_displacement_at_which_first_collision_occurs: float;
    intersection_point: Vector3;

    // iteration tracking for recursive calls
    collision_recursion_depth: s32;
}

world_to_ellipsoid :: (point: Vector3, ellipsoid_radius: Vector3) -> Vector3 {
    return .{
        point.x / ellipsoid_radius.x,
        point.y / ellipsoid_radius.y,
        point.z / ellipsoid_radius.z
    };
}

ellipsoid_to_world :: (point: Vector3, ellipsoid_radius: Vector3) -> Vector3 {
    return .{
        point.x * ellipsoid_radius.x,
        point.y * ellipsoid_radius.y,
        point.z * ellipsoid_radius.z
    };
}

plane_from_triangle :: (tri: Triangle) -> Plane {
    edge1 := tri.p1 - tri.p0;
    edge2 := tri.p2 - tri.p0;
    normal := normalize(cross(edge1, edge2));
    d := -dot(normal, tri.p0);
    return .{normal, d};
}

signed_distance_to_plane :: (point: Vector3, plane: Plane) -> float {
    return dot(point, plane.normal) + plane.d;
}

EPSILON_ :: 0.00001;

// Check collision between a swept unit sphere and one triangle in ellipsoid space.
store_collision_data_between_swept_unit_sphere_and_triangle :: (packet: *Recursive_Collision_Data, tri: Triangle) {
    plane := plane_from_triangle(tri);

    // Fauerby's sample only tests front-facing triangles.
    if dot(plane.normal, packet.e_normalized_displacement) > EPSILON_ {
        return;
    }

    t0, t1: float;
    embedded_in_plane := false;

    signed_dist := signed_distance_to_plane(packet.e_base_point, plane);
    normal_dot_vel := dot(plane.normal, packet.e_displacement);

    if abs(normal_dot_vel) <= EPSILON_ {
        if abs(signed_dist) >= 1.0 {
            return;
        } else {
            embedded_in_plane = true;
            t0 = 0.0;
            t1 = 1.0;
        }
    } else {
        t0 = (-1.0 - signed_dist) / normal_dot_vel;
        t1 = ( 1.0 - signed_dist) / normal_dot_vel;

        if t0 > t1 {
            temp := t0;
            t0 = t1;
            t1 = temp;
        }

        if t0 > 1.0 || t1 < 0.0 {
            return;
        }

        if t0 < 0.0 then t0 = 0.0;
        if t1 < 0.0 then t1 = 0.0;
        if t0 > 1.0 then t0 = 1.0;
        if t1 > 1.0 then t1 = 1.0;
    }

    collision_point: Vector3;
    found_collision := false;
    t := 1.0;

    // Interior triangle collision. If this exists, it is earlier than edge or vertex hits.
    if !embedded_in_plane {
        plane_intersection := packet.e_base_point - plane.normal + t0 * packet.e_displacement;

        if point_in_triangle(plane_intersection, tri) {
            found_collision = true;
            t = t0;
            collision_point = plane_intersection;
        }
    }

    if !found_collision {
        velocity := packet.e_displacement;
        base := packet.e_base_point;
        velocity_sq_len := length_squared(velocity);

        check_vertex :: (base: Vector3, velocity: Vector3, vertex: Vector3, velocity_sq_len: float, t: *float, collision_point: *Vector3) -> bool {
            a := velocity_sq_len;
            b := 2.0 * dot(velocity, base - vertex);
            c := length_squared(vertex - base) - 1.0;

            found, new_t := get_smallest_non_negative_root_of_quadratic(a, b, c, t.*);
            if found {
                t.* = new_t;
                collision_point.* = vertex;
                return true;
            }

            return false;
        };

        found_collision = check_vertex(base, velocity, tri.p0, velocity_sq_len, *t, *collision_point) || found_collision;
        found_collision = check_vertex(base, velocity, tri.p1, velocity_sq_len, *t, *collision_point) || found_collision;
        found_collision = check_vertex(base, velocity, tri.p2, velocity_sq_len, *t, *collision_point) || found_collision;

        check_edge :: (base: Vector3, velocity: Vector3, p1: Vector3, p2: Vector3, velocity_sq_len: float, t: *float, collision_point: *Vector3) -> bool {
            edge := p2 - p1;
            base_to_vertex := p1 - base;

            edge_sq_len := length_squared(edge);
            edge_dot_vel := dot(edge, velocity);
            edge_dot_base_to_vert := dot(edge, base_to_vertex);

            a := edge_sq_len * -velocity_sq_len + edge_dot_vel * edge_dot_vel;
            b := edge_sq_len * (2.0 * dot(velocity, base_to_vertex)) - 2.0 * edge_dot_vel * edge_dot_base_to_vert;
            c := edge_sq_len * (1.0 - length_squared(base_to_vertex)) + edge_dot_base_to_vert * edge_dot_base_to_vert;

            found, new_t := get_smallest_non_negative_root_of_quadratic(a, b, c, t.*);
            if found {
                f := (edge_dot_vel * new_t - edge_dot_base_to_vert) / edge_sq_len;
                if f >= 0.0 && f <= 1.0 {
                    t.* = new_t;
                    collision_point.* = p1 + f * edge;
                    return true;
                }
            }

            return false;
        };

        found_collision = check_edge(base, velocity, tri.p0, tri.p1, velocity_sq_len, *t, *collision_point) || found_collision;
        found_collision = check_edge(base, velocity, tri.p1, tri.p2, velocity_sq_len, *t, *collision_point) || found_collision;
        found_collision = check_edge(base, velocity, tri.p2, tri.p0, velocity_sq_len, *t, *collision_point) || found_collision;
    }

    if found_collision {
        dist_to_collision := t * length(packet.e_displacement);

        if !packet.found_collision || dist_to_collision < packet.parametric_distance_along_displacement_at_which_first_collision_occurs {
            packet.parametric_distance_along_displacement_at_which_first_collision_occurs = dist_to_collision;
            packet.intersection_point = collision_point;
            packet.found_collision = true;
        }
    }
}

VERY_CLOSE_DISTANCE :: 0.005;
MAX_RECURSION_DEPTH :: 5;

Ellipsoid_Collision_Result :: struct {
    new_position: Vector3;
    new_displacement: Vector3;
    on_ground: bool;
    collision_occurred: bool;
}

resolve_collisions_against_body_ellipsoid :: (
    positioned_colliders: Collision_Geometries,
    ellipsoid_radius: Vector3,
    body_position: *Vector3,
    displacement: *Vector3,
    on_ground: *bool,
    gravity_displacement: Vector3 = .{0, 0, 0}
) -> Ellipsoid_Collision_Result {
    on_ground.* = false;

    body_aabb := make_ellipsoid_movement_aabb(body_position.*, displacement.*, gravity_displacement, ellipsoid_radius);

    triangles: [..] Triangle;
    triangles.allocator = temp;

    for 0..positioned_colliders.aabbs.count-1 {
        collider := Collision_Geometry.{positioned_colliders.indexed_triangle_positions[it], positioned_colliders.aabbs[it]};
        if aabbs_are_disjoint(collider.aabb, body_aabb) continue;

        extract_triangles_from_geometry(collider, *triangles);
    }

    if triangles.count == 0 {
        body_position.* += displacement.*;
        body_position.* += gravity_displacement;
        displacement.* += gravity_displacement;

        return .{
            new_position = body_position.*,
            new_displacement = displacement.*,
            on_ground = false,
            collision_occurred = false
        };
    }

    result := collide_and_slide(
        body_position.*,
        displacement.*,
        gravity_displacement,
        ellipsoid_radius,
        triangles
    );

    body_position.* = result.new_position;
    displacement.* = result.new_displacement;
    on_ground.* = result.on_ground;

    return result;
}

collide_and_slide :: (
    position: Vector3,
    displacement: Vector3,
    gravity_displacement: Vector3,
    ellipsoid_radius: Vector3,
    triangles: []Triangle
) -> world_space_result: Ellipsoid_Collision_Result {
    result: Ellipsoid_Collision_Result;

    packet: Recursive_Collision_Data;
    packet.ellipsoid_radius = ellipsoid_radius;

    e_position := world_to_ellipsoid(position, ellipsoid_radius);
    e_displacement := world_to_ellipsoid(displacement, ellipsoid_radius);

    packet.collision_recursion_depth = 0;
    position_after_displacement := collide_with_world(
        *packet,
        e_position,
        e_displacement,
        triangles,
        *result,
        false
    );

    e_gravity_displacement := world_to_ellipsoid(gravity_displacement, ellipsoid_radius);

    packet.collision_recursion_depth = 0;
    final_position := collide_with_world(
        *packet,
        position_after_displacement,
        e_gravity_displacement,
        triangles,
        *result,
        dot(gravity_displacement, Y3) < 0.0
    );

    result.new_position = ellipsoid_to_world(final_position, ellipsoid_radius);
    result.new_displacement = result.new_position - position;

    return result;
}

collide_with_world :: (
    packet: *Recursive_Collision_Data,
    position: Vector3,
    displacement: Vector3,
    triangles: []Triangle,
    result: *Ellipsoid_Collision_Result,
    track_gravity_contact_as_ground: bool
) -> resulting_position_in_espace: Vector3 {
    if packet.collision_recursion_depth > MAX_RECURSION_DEPTH {
        return position;
    }

    if length_squared(displacement) <= EPSILON_ * EPSILON_ {
        return position;
    }

    packet.e_displacement = displacement;
    packet.e_normalized_displacement = normalize(displacement);
    packet.e_base_point = position;
    packet.found_collision = false;
    packet.parametric_distance_along_displacement_at_which_first_collision_occurs = 0.0;

    for tri: triangles {
        e_tri: Triangle;
        e_tri.p0 = world_to_ellipsoid(tri.p0, packet.ellipsoid_radius);
        e_tri.p1 = world_to_ellipsoid(tri.p1, packet.ellipsoid_radius);
        e_tri.p2 = world_to_ellipsoid(tri.p2, packet.ellipsoid_radius);

        store_collision_data_between_swept_unit_sphere_and_triangle(packet, e_tri);
    }

    if !packet.found_collision {
        return position + displacement;
    }

    result.collision_occurred = true;
    if track_gravity_contact_as_ground {
        result.on_ground = true;
    }

    destination := position + displacement;
    new_base_point := position;

    if packet.parametric_distance_along_displacement_at_which_first_collision_occurs >= VERY_CLOSE_DISTANCE {
        v := normalize(displacement) * (packet.parametric_distance_along_displacement_at_which_first_collision_occurs - VERY_CLOSE_DISTANCE);
        new_base_point = packet.e_base_point + v;

        v = normalize(v);
        packet.intersection_point -= VERY_CLOSE_DISTANCE * v;
    }

    slide_plane_origin := packet.intersection_point;
    slide_plane_normal := normalize(new_base_point - packet.intersection_point);

    slide_plane: Plane;
    slide_plane.normal = slide_plane_normal;
    slide_plane.d = -dot(slide_plane_normal, slide_plane_origin);

    distance_to_slide_plane := signed_distance_to_plane(destination, slide_plane);
    new_destination_point := destination - distance_to_slide_plane * slide_plane_normal;
    new_displacement_vector := new_destination_point - packet.intersection_point;

    if length(new_displacement_vector) < VERY_CLOSE_DISTANCE {
        return new_base_point;
    }

    packet.collision_recursion_depth += 1;
    return collide_with_world(packet, new_base_point, new_displacement_vector, triangles, result, track_gravity_contact_as_ground);
}

make_ellipsoid_movement_aabb :: (position: Vector3, velocity: Vector3, gravity_displacement: Vector3, ellipsoid_radius: Vector3) -> AABB {
    min_ := position - ellipsoid_radius;
    max_ := position + ellipsoid_radius;

    end_pos := position + velocity + gravity_displacement;
    end_min := end_pos - ellipsoid_radius;
    end_max := end_pos + ellipsoid_radius;

    result: AABB;
    result.min.x = min(min_.x, end_min.x);
    result.min.y = min(min_.y, end_min.y);
    result.min.z = min(min_.z, end_min.z);
    result.max.x = max(max_.x, end_max.x);
    result.max.y = max(max_.y, end_max.y);
    result.max.z = max(max_.z, end_max.z);

    return result;
}

extract_triangles_from_geometry :: (collider: Collision_Geometry, triangles: *[..]Triangle) {
    indices := collider.indexed_triangle_positions.indices;
    positions := collider.indexed_triangle_positions.positions;

    triangle_count := indices.count / 3;

    for i: 0..triangle_count-1 {
        tri: Triangle;
        tri.p0 = positions[indices[i * 3 + 0]];
        tri.p1 = positions[indices[i * 3 + 1]];
        tri.p2 = positions[indices[i * 3 + 2]];
        array_add(triangles, tri);
    }
}

extract_triangles_from_indexed_geometry :: (vertices: []Vector3, indices: []u32, triangles: *[..]Triangle) {
    triangle_count := indices.count / 3;

    for i: 0..triangle_count-1 {
        tri: Triangle;
        tri.p0 = vertices[indices[i * 3 + 0]];
        tri.p1 = vertices[indices[i * 3 + 1]];
        tri.p2 = vertices[indices[i * 3 + 2]];
        array_add(triangles, tri);
    }
}
