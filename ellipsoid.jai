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

Collision_Packet :: struct {

    // ellipsoid radius (in world space)
    ellipsoid_radius: Vector3;
    
    // information about the move being requested (in ellipsoid space)
    e_velocity: Vector3;
    e_normalized_velocity: Vector3;
    e_base_point: Vector3;
    
    // hit information
    found_collision: bool;
    nearest_distance: float;
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

point_in_triangle :: (point: Vector3, tri: Triangle) -> bool {

    // barycentric coordinates
    edge_to_vertex2 := tri.p2 - tri.p0;
    edge_to_vertex1 := tri.p1 - tri.p0;
    edge_to_point := point - tri.p0;
    
    dot_edge2_edge2 := dot(edge_to_vertex2, edge_to_vertex2);
    dot_edge2_edge1 := dot(edge_to_vertex2, edge_to_vertex1);
    dot_edge2_point := dot(edge_to_vertex2, edge_to_point);
    dot_edge1_edge1 := dot(edge_to_vertex1, edge_to_vertex1);
    dot_edge1_point := dot(edge_to_vertex1, edge_to_point);
    
    inverse_denominator := 1.0 / (dot_edge2_edge2 * dot_edge1_edge1 - dot_edge2_edge1 * dot_edge2_edge1);
    barycentric_u := (dot_edge1_edge1 * dot_edge2_point - dot_edge2_edge1 * dot_edge1_point) * inverse_denominator;
    barycentric_v := (dot_edge2_edge2 * dot_edge1_point - dot_edge2_edge1 * dot_edge2_point) * inverse_denominator;
    
    return (barycentric_u >= 0) && (barycentric_v >= 0) && (barycentric_u + barycentric_v <= 1);
}

// get lowest root of quadratic equation within range [0, max_r]
get_lowest_root :: (a: float, b: float, c: float, max_r: float) -> (found: bool, root: float) {
    determinant := b * b - 4.0 * a * c;
    
    if determinant < 0.0 {
        return false, 0;
    }
    
    sqrt_d := sqrt(determinant);
    r1 := (-b - sqrt_d) / (2.0 * a);
    r2 := (-b + sqrt_d) / (2.0 * a);
    
    // sort so r1 <= r2
    if r1 > r2 {
        temp := r1;
        r1 = r2;
        r2 = temp;
    }
    
    if r1 > 0 && r1 < max_r {
        return true, r1;
    }
    
    if r2 > 0 && r2 < max_r {
        return true, r2;
    }
    
    return false, 0;
}

// check collision between unit sphere and triangle (in ellipsoid space)
check_triangle :: (packet: *Collision_Packet, tri: Triangle) {
    plane := plane_from_triangle(tri);
    
    // only check front-facing triangles
    if dot(plane.normal, packet.e_normalized_velocity) >= 0 {
        return;
    }
    
    // get interval of plane intersection
    t0, t1: float;
    embedded_in_plane := false;
    
    // signed distance from sphere center to plane
    signed_dist := signed_distance_to_plane(packet.e_base_point, plane);
    
    // cache velocity dot normal
    normal_dot_vel := dot(plane.normal, packet.e_velocity);
    
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
        plane_intersection := packet.e_base_point - plane.normal + t0 * packet.e_velocity;
        
        if point_in_triangle(plane_intersection, tri) {
            found_collision = true;
            t = t0;
            collision_point = plane_intersection;
        }
    }
    
    // check collision with vertices and edges
    if !found_collision {
        velocity_sq_len := length_squared(packet.e_velocity);
        
        // helper to check vertex collision
        check_vertex :: (packet: *Collision_Packet, vertex: Vector3, 
                         velocity_sq_len: float, t: *float, 
                         collision_point: *Vector3) -> bool {
            base := packet.e_base_point;
            vel := packet.e_velocity;
            
            // Solve: |base + t*vel - vertex|^2 = 1
            // a*t^2 + b*t + c = 0
            a := velocity_sq_len;
            b := 2.0 * dot(vel, base - vertex);
            c := length_squared(vertex - base) - 1.0;
            
            found, new_t := get_lowest_root(a, b, c, <<t);
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
        check_edge :: (packet: *Collision_Packet, p1: Vector3, p2: Vector3,
                       velocity_sq_len: float, t: *float,
                       collision_point: *Vector3) -> bool {
            base := packet.e_base_point;
            vel := packet.e_velocity;
            
            edge := p2 - p1;
            base_to_vertex := p1 - base;
            
            edge_sq_len := length_squared(edge);
            edge_dot_vel := dot(edge, vel);
            edge_dot_base_to_vert := dot(edge, base_to_vertex);
            
            // parametric equation for closest point on edge
            a := edge_sq_len * (-velocity_sq_len) + edge_dot_vel * edge_dot_vel;
            b := edge_sq_len * (2.0 * dot(vel, base_to_vertex)) - 2.0 * edge_dot_vel * edge_dot_base_to_vert;
            c := edge_sq_len * (1.0 - length_squared(base_to_vertex)) + edge_dot_base_to_vert * edge_dot_base_to_vert;
            
            found, new_t := get_lowest_root(a, b, c, <<t);
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
        dist_to_collision := t * length(packet.e_velocity);
        
        if !packet.found_collision || dist_to_collision < packet.nearest_distance {
            packet.nearest_distance = dist_to_collision;
            packet.intersection_point = collision_point;
            packet.found_collision = true;
        }
    }
}

// collision response (sliding)

VERY_CLOSE_DISTANCE :: 0.005;
MAX_RECURSION_DEPTH :: 5;

// main collision and slide function
collide_and_slide :: (position: Vector3, velocity: Vector3, 
                      ellipsoid_radius: Vector3, triangles: []Triangle) -> Vector3 {
    packet: Collision_Packet;
    packet.ellipsoid_radius = ellipsoid_radius;
    packet.collision_recursion_depth = 0;
    
    e_position := world_to_ellipsoid(position, ellipsoid_radius);
    e_velocity := world_to_ellipsoid(velocity, ellipsoid_radius);
    
    // do collision detection and response
    final_position := collide_with_world(*packet, e_position, e_velocity, triangles);
    
    return ellipsoid_to_world(final_position, ellipsoid_radius);
}

// recursive collision response
collide_with_world :: (packet: *Collision_Packet, position: Vector3, 
                       velocity: Vector3, triangles: []Triangle) -> Vector3 {

    if packet.collision_recursion_depth > MAX_RECURSION_DEPTH {
        return position;
    }
    
    // setup packet for this iteration
    packet.e_velocity = velocity;
    packet.e_normalized_velocity = normalize(velocity);
    packet.e_base_point = position;
    packet.found_collision = false;
    packet.nearest_distance = 0;
    
    // convert triangles to ellipsoid space and check each one
    for tri: triangles {
        e_tri: Triangle;
        e_tri.p0 = world_to_ellipsoid(tri.p0, packet.ellipsoid_radius);
        e_tri.p1 = world_to_ellipsoid(tri.p1, packet.ellipsoid_radius);
        e_tri.p2 = world_to_ellipsoid(tri.p2, packet.ellipsoid_radius);
        check_triangle(packet, e_tri);
    }
    
    if !packet.found_collision {
        return position + velocity;
    }

    // now we know that a collision did occur and so we compute the responce.
    
    destination := position + velocity;
    new_base_point := position;
    
    // move to collision point (with small offset)
    if packet.nearest_distance >= VERY_CLOSE_DISTANCE {
        v := normalize(velocity) * (packet.nearest_distance - VERY_CLOSE_DISTANCE);
        new_base_point = position + v;
        
        // adjust intersection point
        v = normalize(v);
        packet.intersection_point = packet.intersection_point - VERY_CLOSE_DISTANCE * v;
    }
    
    // calculate sliding plane
    slide_plane_origin := packet.intersection_point;
    slide_plane_normal := normalize(new_base_point - packet.intersection_point);
    
    slide_plane: Plane;
    slide_plane.normal = slide_plane_normal;
    slide_plane.d = -dot(slide_plane_normal, slide_plane_origin);
    
    // project destination onto sliding plane
    dist := signed_distance_to_plane(destination, slide_plane);
    new_destination := destination - dist * slide_plane_normal;
    
    // calculate new velocity vector
    new_velocity := new_destination - packet.intersection_point;
    
    // check for very small velocity (avoid infinite recursion)
    if length(new_velocity) < VERY_CLOSE_DISTANCE {
        return new_base_point;
    }
    
    // recurse
    packet.collision_recursion_depth += 1;
    return collide_with_world(packet, new_base_point, new_velocity, triangles);
}


// apply gravity with collision
collide_and_slide_with_gravity :: (position: Vector3, velocity: Vector3, gravity: Vector3,
                                    ellipsoid_radius: Vector3, triangles: []Triangle) -> Vector3 {
    // First, handle horizontal movement
    pos := collide_and_slide(position, velocity, ellipsoid_radius, triangles);
    
    // Then handle gravity
    pos = collide_and_slide(pos, gravity, ellipsoid_radius, triangles);
    
    return pos;
}


// helper to create a vector faster that probably doesn't have to exist but does because it's actually useful below to save keystrokes.
v3 :: (x: float, y: float, z: float) -> Vector3 {
    return .{x, y, z};
}

test :: () {
    ellipsoid_radius := v3(0.5, 1.0, 0.5);
    
    player_position := v3(0, 5, 0);
    player_velocity := v3(0.1, 0, 0);  // slower horizontal movement
    gravity := v3(0, -0.1, 0);
    
    triangles: [2]Triangle;
    triangles[0] = .{
        v3(-10, 0, -10),
        v3(10, 0, 10),      
        v3(10, 0, -10)
    };
    triangles[1] = .{
        v3(-10, 0, -10),
        v3(-10, 0, 10),     
        v3(10, 0, 10)
    };
    
    for frame: 0..60 {
        player_position = collide_and_slide_with_gravity(
            player_position, 
            player_velocity,
            gravity,
            ellipsoid_radius, 
            triangles
        );
        
        print("Frame %: Position = (%, %, %)\n", 
              frame, player_position.x, player_position.y, player_position.z);
    }
}


Ellipsoid_Collision_Result :: struct {
    new_position: Vector3;
    new_velocity: Vector3;
    on_ground: bool;
    collision_occurred: bool;
    ground_normal: Vector3;
}


// this is the function that you call to do everything.
resolve_collisions_against_body_ellipsoid :: (
    positioned_colliders: Collision_Geometries, 
    ellipsoid_radius: Vector3,
    body_position: *Vector3, 
    desired_body_position_offset: *Vector3,  // rename to body frame velocity
    gravity: Vector3, // rename to frame gravity
    on_ground: *bool
) {
    on_ground.* = false;
    
    // expand body aabb by velocity + ellipsoid radius for broadphase
    body_aabb := make_ellipsoid_movement_aabb(body_position.*, desired_body_position_offset.*, gravity, ellipsoid_radius);
    
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
        body_position.* += desired_body_position_offset.*;
        body_position.* += gravity;
        return;
    }
    
    result := collide_and_slide_full(
        body_position.*,
        desired_body_position_offset.*,
        gravity,
        ellipsoid_radius,
        triangles
    );
    
    body_position.* = result.new_position;
    desired_body_position_offset.* = result.new_velocity;
    on_ground.* = result.on_ground;
}


collide_and_slide_full :: (
    position: Vector3, 
    velocity: Vector3, 
    gravity: Vector3,
    ellipsoid_radius: Vector3, 
    triangles: []Triangle
) -> Ellipsoid_Collision_Result {
    
    result: Ellipsoid_Collision_Result;
    result.on_ground = false;
    result.collision_occurred = false;
    
    // phase 1: apply horizontal/intended movement
    packet: Collision_Packet;
    packet.ellipsoid_radius = ellipsoid_radius;
    packet.collision_recursion_depth = 0;
    
    e_position := world_to_ellipsoid(position, ellipsoid_radius);
    e_velocity := world_to_ellipsoid(velocity, ellipsoid_radius);
    
    pos_after_velocity := collide_with_world_tracking(
        *packet, 
        e_position, 
        e_velocity, 
        triangles,
        *result
    );
    
    // phase 2: apply gravity
    packet.collision_recursion_depth = 0;
    e_gravity := world_to_ellipsoid(gravity, ellipsoid_radius);
    
    final_e_position := collide_with_world_tracking(
        *packet,
        pos_after_velocity,
        e_gravity,
        triangles,
        *result
    );
    
    result.new_position = ellipsoid_to_world(final_e_position, ellipsoid_radius);
    
    // adjust velocity based on collision
    if result.collision_occurred {
        // remove velocity component going into ground
        if result.on_ground {
            vel_dot_normal := dot(velocity, result.ground_normal);
            if vel_dot_normal < 0 {
                result.new_velocity = velocity - vel_dot_normal * result.ground_normal;
            } else {
                result.new_velocity = velocity;
            }
        } else {
            result.new_velocity = velocity;
        }
    } else {
        result.new_velocity = velocity;
    }
    
    return result;
}


collide_with_world_tracking :: (
    packet: *Collision_Packet, 
    position: Vector3, 
    velocity: Vector3, 
    triangles: []Triangle,
    result: *Ellipsoid_Collision_Result
) -> Vector3 {
    
    if packet.collision_recursion_depth > MAX_RECURSION_DEPTH {
        return position;
    }
    
    // setup packet
    packet.e_velocity = velocity;
    packet.e_normalized_velocity = normalize(velocity);
    packet.e_base_point = position;
    packet.found_collision = false;
    packet.nearest_distance = 0;
    
    // check triangles
    for tri: triangles {
        e_tri: Triangle;
        e_tri.p0 = world_to_ellipsoid(tri.p0, packet.ellipsoid_radius);
        e_tri.p1 = world_to_ellipsoid(tri.p1, packet.ellipsoid_radius);
        e_tri.p2 = world_to_ellipsoid(tri.p2, packet.ellipsoid_radius);
        check_triangle(packet, e_tri);
    }
    
    if !packet.found_collision {
        return position + velocity;
    }
    
    // collision occurred
    result.collision_occurred = true;
    
    destination := position + velocity;
    new_base_point := position;
    
    if packet.nearest_distance >= VERY_CLOSE_DISTANCE {
        v := normalize(velocity) * (packet.nearest_distance - VERY_CLOSE_DISTANCE);
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
    new_destination := destination - dist * slide_plane_normal;
    new_velocity := new_destination - packet.intersection_point;
    
    if length(new_velocity) < VERY_CLOSE_DISTANCE {
        return new_base_point;
    }
    
    packet.collision_recursion_depth += 1;
    return collide_with_world_tracking(packet, new_base_point, new_velocity, triangles, result);
}

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
