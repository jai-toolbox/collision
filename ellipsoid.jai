/*
implementation of kasper fauerby's "improved collision detection and response"
http://www.peroxide.dk/papers/collision/collision.pdf

The general idea is that you can represent your character as an ellipsoid, the moment you do that
then you can realize that via a scaling you can turn your guy back into unit sphere, while also adjusting
the scale of the geometry as well. This is called the ellipsoid space and it's beneficial to work in this
space because then doing collisions against your sphere is much simpler.
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

PLANE_FRONT :: 0;
PLANE_BACK  :: 1;
PLANE_ON    :: 2;

EPSILON_ :: 0.00001;

classify_point_ :: (point: Vector3, plane: Plane) -> s32 {
    dist := signed_distance_to_plane(point, plane);
    if dist > EPSILON_ return PLANE_FRONT;
    if dist < -EPSILON_ return PLANE_BACK;
    return PLANE_ON;
}

// check collision between unit sphere and triangle (in ellipsoid space)
store_collision_data_between_swept_unit_sphere_and_triangle :: (packet: *Recursive_Collision_Data, tri: Triangle) {
    plane := plane_from_triangle(tri);
    
    // only check front-facing triangles
    if dot(plane.normal, packet.e_normalized_displacement) >= 0 {
        return;
    }
    
    // get interval of plane intersection
    t0, t1: float;
    embedded_in_plane := false;
    
    // signed distance from sphere center to plane
    signed_dist := signed_distance_to_plane(packet.e_base_point, plane);
    
    // cache velocity dot normal
    normal_dot_vel := dot(plane.normal, packet.e_displacement);
    
    // sphere traveling parallel to plane
    if normal_dot_vel == 0 {
        if abs(signed_dist) >= 1.0 {
            // Sphere not embedded, no collision possible
            return;
        } else {
            // Sphere embedded in plane
            embedded_in_plane = true;
            t0 = 0.0;
            t1 = 1.0;
        }
    } else {
        // Calculate intersection interval
        t0 = (-1.0 - signed_dist) / normal_dot_vel;
        t1 = (1.0 - signed_dist) / normal_dot_vel;
        
        // Swap so t0 < t1
        if t0 > t1 {
            temp := t0;
            t0 = t1;
            t1 = temp;
        }
        
        // check that at least one result is within range
        if t0 > 1.0 || t1 < 0.0 {
            return;  // No collision possible
        }
        
        // clamp to [0, 1]
        if t0 < 0.0 then t0 = 0.0;
        if t1 > 1.0 then t1 = 1.0;
    }
    
    // collision detection starts now.
    collision_point: Vector3;
    found_collision := false;
    t := 1.0;
    
    // check collision with triangle interior
    if !embedded_in_plane {
        plane_intersection := packet.e_base_point - plane.normal + t0 * packet.e_displacement;
        
        if point_in_triangle(plane_intersection, tri) {
            found_collision = true;
            t = t0;
            collision_point = plane_intersection;
        }
    }
    
    // check collision with vertices and edges
    if !found_collision {
        velocity_sq_len := length_squared(packet.e_displacement);
        
        // helper to check vertex collision
        check_vertex :: (packet: *Recursive_Collision_Data, vertex: Vector3, 
                         velocity_sq_len: float, t: *float, 
                         collision_point: *Vector3) -> bool {
            base := packet.e_base_point;
            vel := packet.e_displacement;
            
            // Solve: |base + t*vel - vertex|^2 = 1
            // a*t^2 + b*t + c = 0
            a := velocity_sq_len;
            b := 2.0 * dot(vel, base - vertex);
            c := length_squared(vertex - base) - 1.0;
            
            found, new_t := get_smallest_non_negative_root_of_quadratic(a, b, c, <<t);
            if found {
                <<t = new_t;
                <<collision_point = vertex;
                return true;
            }
            return false;
        };
        
        found_collision = check_vertex(packet, tri.p0, velocity_sq_len, *t, *collision_point) || found_collision;
        found_collision = check_vertex(packet, tri.p1, velocity_sq_len, *t, *collision_point) || found_collision;
        found_collision = check_vertex(packet, tri.p2, velocity_sq_len, *t, *collision_point) || found_collision;
        
        // helper to check edge collision
        check_edge :: (packet: *Recursive_Collision_Data, p1: Vector3, p2: Vector3,
                       velocity_sq_len: float, t: *float,
                       collision_point: *Vector3) -> bool {
            base := packet.e_base_point;
            vel := packet.e_displacement;
            
            edge := p2 - p1;
            base_to_vertex := p1 - base;
            
            edge_sq_len := length_squared(edge);
            edge_dot_vel := dot(edge, vel);
            edge_dot_base_to_vert := dot(edge, base_to_vertex);
            
            // parametric equation for closest point on edge
            a := edge_sq_len * (-velocity_sq_len) + edge_dot_vel * edge_dot_vel;
            b := edge_sq_len * (2.0 * dot(vel, base_to_vertex)) - 2.0 * edge_dot_vel * edge_dot_base_to_vert;
            c := edge_sq_len * (1.0 - length_squared(base_to_vertex)) + edge_dot_base_to_vert * edge_dot_base_to_vert;
            
            found, new_t := get_smallest_non_negative_root_of_quadratic(a, b, c, <<t);
            if found {
                // check if intersection is within edge segment
                f := (edge_dot_vel * new_t - edge_dot_base_to_vert) / edge_sq_len;
                if f >= 0.0 && f <= 1.0 {
                    <<t = new_t;
                    <<collision_point = p1 + f * edge;
                    return true;
                }
            }
            return false;
        };
        
        found_collision = check_edge(packet, tri.p0, tri.p1, velocity_sq_len, *t, *collision_point) || found_collision;
        found_collision = check_edge(packet, tri.p1, tri.p2, velocity_sq_len, *t, *collision_point) || found_collision;
        found_collision = check_edge(packet, tri.p2, tri.p0, velocity_sq_len, *t, *collision_point) || found_collision;
    }
    
    // update collision packet
    if found_collision {
        dist_to_collision := t * length(packet.e_displacement);
        
        if !packet.found_collision || dist_to_collision < packet.parametric_distance_along_displacement_at_which_first_collision_occurs {
            packet.parametric_distance_along_displacement_at_which_first_collision_occurs = dist_to_collision;
            packet.intersection_point = collision_point;
            packet.found_collision = true;
        }
    }
}

// collision response (sliding)

VERY_CLOSE_DISTANCE :: 0.005;
MAX_RECURSION_DEPTH :: 5;


// helper to create a vector faster that probably doesn't have to exist but does because it's actually useful below to save keystrokes.
v3 :: (x: float, y: float, z: float) -> Vector3 {
    return .{x, y, z};
}


// Note the internal values may be in espace
Ellipsoid_Collision_Result :: struct {
    new_position: Vector3;
    new_displacement: Vector3;
    on_ground: bool;
    collision_occurred: bool;
    ground_normal: Vector3;
}


// this is the function that you call to do everything.
// collide and slide until fully displaced or something.
// in the future it might be good to also pass in the real velocity that the user has
// and then when they collide with something we can "subtract off" the part going into the collision, the way it is done with the old collision system
// this would stop players from building up velocity on a wall, but at the same time, that is a little bit fun.

resolve_collisions_against_body_ellipsoid :: (
    positioned_colliders: Collision_Geometries, 
    ellipsoid_radius: Vector3,
    body_position: *Vector3, 
    displacement: *Vector3,  // rename to body frame velocity
    on_ground: *bool,
    gravity: Vector3 = .{0, 0, 0} // rename to frame gravity or do something else...
) {
    on_ground.* = false;
    
    // expand body aabb by velocity + ellipsoid radius for broadphase
    body_aabb := make_ellipsoid_movement_aabb(body_position.*, displacement.*, gravity, ellipsoid_radius);
    
    // collect triangles from colliders that pass broadphase
    triangles: [..] Triangle;
    triangles.allocator = temp;
    
    for 0..positioned_colliders.aabbs.count-1 {
        collider := Collision_Geometry.{positioned_colliders.indexed_triangle_positions[it], positioned_colliders.aabbs[it]};
        if aabbs_are_disjoint(collider.aabb, body_aabb) {
            continue;
        }
        
        extract_triangles_from_geometry(collider, *triangles);
    }
    
    didnt_collide_with_anything := triangles.count == 0;
    if didnt_collide_with_anything {
        body_position.* += displacement.*;
        body_position.* += gravity;
        return;
    }
    
    result := collide_and_slide(
        body_position.*,
        displacement.*,
        gravity,
        ellipsoid_radius,
        triangles
    );
    
    body_position.* = result.new_position;
    // displacement.* = result.new_displacement;
    on_ground.* = result.on_ground;
}


collide_and_slide :: (
    position: Vector3, // rename to initial position
    displacement: Vector3, 
    gravity: Vector3,
    ellipsoid_radius: Vector3, 
    triangles: []Triangle
) -> world_space_result: Ellipsoid_Collision_Result {
    
    result: Ellipsoid_Collision_Result;
    result.on_ground = false;
    result.collision_occurred = false;
    
    // phase 1: apply horizontal/intended movement
    packet: Recursive_Collision_Data;
    packet.ellipsoid_radius = ellipsoid_radius;
    packet.collision_recursion_depth = 0;
    
    e_position := world_to_ellipsoid(position, ellipsoid_radius);
    e_displacement := world_to_ellipsoid(displacement, ellipsoid_radius);
    
    pos_after_displacement := collide_and_slide_recursive(
        *packet, 
        e_position, 
        e_displacement, 
        triangles,
        *result
    );
    
    // phase 2: apply gravity, not doing this yet.
    // packet.collision_recursion_depth = 0;
    // e_gravity := world_to_ellipsoid(gravity, ellipsoid_radius);
    // 
    // final_e_position := collide_and_slide_recursive(
    //     *packet,
    //     pos_after_displacement,
    //     e_gravity,
    //     triangles,
    //     *result
    // );
    
    // result.new_position = ellipsoid_to_world(final_e_position, ellipsoid_radius);
    result.new_position = ellipsoid_to_world(pos_after_displacement, ellipsoid_radius);
    
    // adjust displacement based on collision this is just wrong, but the purpose was because at one time you wer passing in dt * vel, and then you'd take
    // the result and then div by dt to get the new vel, which felt weird because we use the dt * vel + 0.5 * a dt^2 model.
    // if result.collision_occurred { 
    //     // remove displacement component going into ground
    //     if result.on_ground {
    //         vel_dot_normal := dot(displacement, result.ground_normal);
    //         if vel_dot_normal < 0 {
    //             result.new_displacement = displacement - vel_dot_normal * result.ground_normal;
    //         } else {
    //             result.new_displacement = displacement;
    //         }
    //     } else {
    //         result.new_displacement = displacement;
    //     }
    // } else {
    //     result.new_displacement = displacement;
    // }
    
    return result;
}


collide_and_slide_recursive :: (
    packet: *Recursive_Collision_Data, 
    position: Vector3, 
    displacement: Vector3, 
    triangles: []Triangle,
    result: *Ellipsoid_Collision_Result
) -> resulting_position_in_espace: Vector3 {
    
    if packet.collision_recursion_depth > MAX_RECURSION_DEPTH {
        return position;
    }
    
    // setup packet
    packet.e_displacement = displacement;
    packet.e_normalized_displacement = normalize(displacement);
    packet.e_base_point = position;
    packet.found_collision = false;
    packet.parametric_distance_along_displacement_at_which_first_collision_occurs = 0;
    
    // check triangles
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
    
    // collision occurred
    result.collision_occurred = true;
    
    destination := position + displacement;
    new_base_point := position;
    
    // if first collision happens far away, then update the base point
    if packet.parametric_distance_along_displacement_at_which_first_collision_occurs >= VERY_CLOSE_DISTANCE {
        v := normalize(displacement) * (packet.parametric_distance_along_displacement_at_which_first_collision_occurs - VERY_CLOSE_DISTANCE);
        new_base_point = position + v;
        v = normalize(v);
        packet.intersection_point = packet.intersection_point - VERY_CLOSE_DISTANCE * v;
    }
    
    // calculate sliding plane normal (this is the collision normal)
    slide_plane_origin := packet.intersection_point;
    slide_plane_normal := normalize(new_base_point - packet.intersection_point);
    
    // convert normal back to world space for ground check
    world_normal := normalize(ellipsoid_to_world(slide_plane_normal, packet.ellipsoid_radius));
    
    // check if this is ground (normal pointing up)
    cos_45 :: 0.707;
    up_alignment := dot(world_normal, v3(0, 1, 0));
    
    if up_alignment > cos_45 {
        result.on_ground = true;
        result.ground_normal = world_normal;
    }
    
    slide_plane: Plane;
    slide_plane.normal = slide_plane_normal;
    slide_plane.d = -dot(slide_plane_normal, slide_plane_origin);
    
    dist := signed_distance_to_plane(destination, slide_plane);
    destination_projected_onto_slide_plane := destination - dist * slide_plane_normal;
    remaining_displacement := destination_projected_onto_slide_plane - packet.intersection_point;
    

    if length(remaining_displacement) < VERY_CLOSE_DISTANCE {
        return new_base_point; // ending recursion because we travelled enough.
    }

    // otherwise we still have to displace more, so continue recursion
    
    packet.collision_recursion_depth += 1;
    return collide_and_slide_recursive(packet, new_base_point, remaining_displacement, triangles, result);
}

// rename to swept ellipsoid aabb in the future
make_ellipsoid_movement_aabb :: (position: Vector3, velocity: Vector3, gravity: Vector3, ellipsoid_radius: Vector3) -> AABB {
    // start with ellipsoid at current position
    min_ := position - ellipsoid_radius;
    max_ := position + ellipsoid_radius;
    
    // expand to include destination
    end_pos := position + velocity + gravity;
    end_min := end_pos - ellipsoid_radius;
    end_max := end_pos + ellipsoid_radius;
    
    // union of both aabbs
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

// if your geometry uses indices instead:
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
