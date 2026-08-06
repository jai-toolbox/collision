// in the future I don't want to depend on all of geom, because all I need is the itp type doing it for now.
#import "tbx/geometry";
#import "tbx/model_loading";
#import "tbx/physics";

#load "axis_aligned_bounding_box.jai";
#load "ellipsoid.jai";
#load "kinematic_character.jai";

#import "File";
#import "File_Utilities";
#import "Math";
#import "Basic";
#import "String";


// good to use when there's just like one of them.
Collision_Geometry :: struct {
    indexed_triangle_positions : Indexed_Triangle_Positions;
    aabb : AABB;
}

deinit :: (using geo: *Collision_Geometry) {
    deinit(*indexed_triangle_positions);
}

Kinematic_Collision_Triangle_Ref :: struct {
    mesh_index: int;
    triangle_index: int;
}

Kinematic_Collision_Grid_Cell :: struct {
    triangle_refs: [..] Kinematic_Collision_Triangle_Ref;
}

Kinematic_Collision_Grid :: struct {
    built := false;
    cell_size: float = 2.0;
    bounds: AABB;
    cells_x, cells_y, cells_z: int;
    cells: [..] Kinematic_Collision_Grid_Cell;
}

CHARACTER_VIRTUAL_COLLISION_CACHE_MAGIC :: 0x43564d31; // "CVM1"
CHARACTER_VIRTUAL_COLLISION_CACHE_VERSION :: 1;

Character_Virtual_Collision_Mesh_Cache_Header :: struct {
    magic: u32;
    version: u32;
    physics_max_geometry_triangles: int;
    triangle_count: int;
    triangle_checksum: u64;
    mesh_size: int;
}

// good to use when you need to iterate over each efficiently
Collision_Geometries :: struct {
    indexed_triangle_positions : [..] Indexed_Triangle_Positions;
    aabbs : [..] AABB;
    kinematic_grid: Kinematic_Collision_Grid;
    character_virtual_triangles: [..] Geometry_Triangle;
    character_virtual_mesh_built := false;
}

deinit :: (using geos: *Collision_Geometries) {
    for *indexed_triangle_positions deinit(it);
    array_free(indexed_triangle_positions);
    array_free(aabbs);
    deinit(*kinematic_grid);
    array_free(character_virtual_triangles);
    character_virtual_mesh_built = false;
}

deinit :: (grid: *Kinematic_Collision_Grid) {
    for * grid.cells array_free(it.triangle_refs);
    array_free(grid.cells);
    grid.built = false;
}

collision_triangle_aabb :: (p0: Vector3, p1: Vector3, p2: Vector3) -> AABB {
    return .{min = min(min(p0, p1), p2), max = max(max(p0, p1), p2)};
}

collision_grid_clamp_cell :: (v: int, max_value: int) -> int {
    if v < 0 return 0;
    if v >= max_value return max_value - 1;
    return v;
}

collision_grid_cell_for_position :: (grid: *Kinematic_Collision_Grid, p: Vector3) -> int, int, int {
    x := collision_grid_clamp_cell(cast(int) floor((p.x - grid.bounds.min.x) / grid.cell_size), grid.cells_x);
    y := collision_grid_clamp_cell(cast(int) floor((p.y - grid.bounds.min.y) / grid.cell_size), grid.cells_y);
    z := collision_grid_clamp_cell(cast(int) floor((p.z - grid.bounds.min.z) / grid.cell_size), grid.cells_z);
    return x, y, z;
}

collision_grid_index :: (grid: *Kinematic_Collision_Grid, x: int, y: int, z: int) -> int {
    return x + y * grid.cells_x + z * grid.cells_x * grid.cells_y;
}

collision_grid_add_triangle :: (grid: *Kinematic_Collision_Grid, tri_aabb: AABB, ref: Kinematic_Collision_Triangle_Ref) {
    min_x, min_y, min_z := collision_grid_cell_for_position(grid, tri_aabb.min);
    max_x, max_y, max_z := collision_grid_cell_for_position(grid, tri_aabb.max);

    for z: min_z..max_z {
        for y: min_y..max_y {
            for x: min_x..max_x {
                cell_index := collision_grid_index(grid, x, y, z);
                array_add(*grid.cells[cell_index].triangle_refs, ref);
            }
        }
    }
}

build_kinematic_collision_grid :: (geos: *Collision_Geometries) {
    deinit(*geos.kinematic_grid);
    if geos.aabbs.count == 0 return;

    bounds := geos.aabbs[0];
    for i: 1..geos.aabbs.count - 1 {
        bounds.min = min(bounds.min, geos.aabbs[i].min);
        bounds.max = max(bounds.max, geos.aabbs[i].max);
    }

    // Keep the first pass conservative: avoid dense-grid blowups on very large maps.
    CELL_SIZE :: 2.0;
    MAX_CELLS :: 262144;
    size := bounds.max - bounds.min;
    cells_x := max(1, cast(int) ceil(size.x / CELL_SIZE));
    cells_y := max(1, cast(int) ceil(size.y / CELL_SIZE));
    cells_z := max(1, cast(int) ceil(size.z / CELL_SIZE));
    total_cells := cells_x * cells_y * cells_z;
    if total_cells <= 0 || total_cells > MAX_CELLS {
        log("kinematic collision grid skipped: cells=% dims=(%, %, %)", total_cells, cells_x, cells_y, cells_z);
        return;
    }

    grid := *geos.kinematic_grid;
    grid.cell_size = CELL_SIZE;
    grid.bounds = bounds;
    grid.cells_x = cells_x;
    grid.cells_y = cells_y;
    grid.cells_z = cells_z;
    array_resize(*grid.cells, total_cells);

    for itp, mesh_index: geos.indexed_triangle_positions {
        tri_count := itp.indices.count / 3;
        for tri_index: 0..tri_count - 1 {
            p0 := itp.positions[itp.indices[tri_index * 3 + 0]];
            p1 := itp.positions[itp.indices[tri_index * 3 + 1]];
            p2 := itp.positions[itp.indices[tri_index * 3 + 2]];
            collision_grid_add_triangle(grid, collision_triangle_aabb(p0, p1, p2), .{mesh_index = mesh_index, triangle_index = tri_index});
        }
    }

    grid.built = true;
    log("kinematic collision grid built: cells=% dims=(%, %, %) cell_size=%", total_cells, cells_x, cells_y, cells_z, CELL_SIZE);
}

character_virtual_collision_cache_path :: (map_path: string) -> string {
    return tprint("%.cvmesh", map_path);
}

character_virtual_collision_checksum_mix :: (hash: u64, value: u32) -> u64 {
    FNV_PRIME :: 1099511628211;
    return (hash ^ cast(u64) value) * FNV_PRIME;
}

character_virtual_collision_triangle_checksum :: (triangles: [] Geometry_Triangle) -> u64 {
    hash: u64 = 1469598103934665603;
    for tri: triangles {
        hash = character_virtual_collision_checksum_mix(hash, float32_to_bits(cast(float32) tri.a.x));
        hash = character_virtual_collision_checksum_mix(hash, float32_to_bits(cast(float32) tri.a.y));
        hash = character_virtual_collision_checksum_mix(hash, float32_to_bits(cast(float32) tri.a.z));
        hash = character_virtual_collision_checksum_mix(hash, float32_to_bits(cast(float32) tri.b.x));
        hash = character_virtual_collision_checksum_mix(hash, float32_to_bits(cast(float32) tri.b.y));
        hash = character_virtual_collision_checksum_mix(hash, float32_to_bits(cast(float32) tri.b.z));
        hash = character_virtual_collision_checksum_mix(hash, float32_to_bits(cast(float32) tri.c.x));
        hash = character_virtual_collision_checksum_mix(hash, float32_to_bits(cast(float32) tri.c.y));
        hash = character_virtual_collision_checksum_mix(hash, float32_to_bits(cast(float32) tri.c.z));
    }
    return hash;
}

try_load_character_virtual_collision_mesh_cache :: (geos: *Collision_Geometries, map_path: string, triangle_checksum: u64) -> bool {
    cache_path := character_virtual_collision_cache_path(map_path);
    data, ok := read_entire_file(cache_path, log_errors = false);
    if !ok return false;
    defer free(data);

    expected_size := size_of(Character_Virtual_Collision_Mesh_Cache_Header) + size_of(Physics_Character_Virtual_Triangle_Mesh);
    if data.count != expected_size return false;

    header := (cast(*Character_Virtual_Collision_Mesh_Cache_Header) data.data).*;
    if header.magic != CHARACTER_VIRTUAL_COLLISION_CACHE_MAGIC return false;
    if header.version != CHARACTER_VIRTUAL_COLLISION_CACHE_VERSION return false;
    if header.physics_max_geometry_triangles != PHYSICS_MAX_GEOMETRY_TRIANGLES return false;
    if header.triangle_count != geos.character_virtual_triangles.count return false;
    if header.triangle_checksum != triangle_checksum return false;
    if header.mesh_size != size_of(Physics_Character_Virtual_Triangle_Mesh) return false;

    mesh_data := cast(*u8) data.data + size_of(Character_Virtual_Collision_Mesh_Cache_Header);
    memcpy(*character_virtual_global_mesh, mesh_data, size_of(Physics_Character_Virtual_Triangle_Mesh));
    character_virtual_global_mesh.triangles = geos.character_virtual_triangles;
    geos.character_virtual_mesh_built = character_virtual_global_mesh.valid;
    if !geos.character_virtual_mesh_built return false;

    log("character virtual collision mesh loaded from cache '%': triangles=% nodes=%",
        cache_path,
        geos.character_virtual_triangles.count,
        character_virtual_global_mesh.node_count);
    return true;
}

write_character_virtual_collision_mesh_cache :: (geos: *Collision_Geometries, map_path: string, triangle_checksum: u64) {
    cache_path := character_virtual_collision_cache_path(map_path);
    cache_directory := path_strip_filename(cache_path);
    if cache_directory.count > 0 make_directory_if_it_does_not_exist(cache_directory, recursive = true);

    header := Character_Virtual_Collision_Mesh_Cache_Header.{
        magic = CHARACTER_VIRTUAL_COLLISION_CACHE_MAGIC,
        version = CHARACTER_VIRTUAL_COLLISION_CACHE_VERSION,
        physics_max_geometry_triangles = PHYSICS_MAX_GEOMETRY_TRIANGLES,
        triangle_count = geos.character_virtual_triangles.count,
        triangle_checksum = triangle_checksum,
        mesh_size = size_of(Physics_Character_Virtual_Triangle_Mesh),
    };

    bytes := size_of(Character_Virtual_Collision_Mesh_Cache_Header) + size_of(Physics_Character_Virtual_Triangle_Mesh);
    memory := alloc(bytes);
    defer free(memory);

    memcpy(memory, *header, size_of(Character_Virtual_Collision_Mesh_Cache_Header));
    memcpy(cast(*u8) memory + size_of(Character_Virtual_Collision_Mesh_Cache_Header), *character_virtual_global_mesh, size_of(Physics_Character_Virtual_Triangle_Mesh));

    ok := write_entire_file(cache_path, memory, bytes);
    if ok {
        log("character virtual collision mesh cache written '%': bytes=%", cache_path, bytes);
    } else {
        log_error("failed to write character virtual collision mesh cache '%'", cache_path);
    }
}

build_character_virtual_collision_mesh :: (geos: *Collision_Geometries, map_path: string) {
    geos.character_virtual_mesh_built = false;
    array_free(geos.character_virtual_triangles);

    triangle_count := 0;
    for itp: geos.indexed_triangle_positions triangle_count += itp.indices.count / 3;
    if triangle_count <= 0 return;

    if triangle_count > PHYSICS_MAX_GEOMETRY_TRIANGLES {
        assert(false, tprint("character virtual collision mesh triangle budget exceeded: triangles=% max=%", triangle_count, PHYSICS_MAX_GEOMETRY_TRIANGLES));
    }

    array_reserve(*geos.character_virtual_triangles, triangle_count);
    for itp: geos.indexed_triangle_positions {
        tri_count := itp.indices.count / 3;
        for tri_index: 0..tri_count - 1 {
            i0 := cast(int) itp.indices[tri_index * 3 + 0];
            i1 := cast(int) itp.indices[tri_index * 3 + 1];
            i2 := cast(int) itp.indices[tri_index * 3 + 2];
            array_add(*geos.character_virtual_triangles, .{
                a = itp.positions[i0],
                b = itp.positions[i1],
                c = itp.positions[i2],
            });
        }
    }

    triangle_checksum := character_virtual_collision_triangle_checksum(geos.character_virtual_triangles);
    if try_load_character_virtual_collision_mesh_cache(geos, map_path, triangle_checksum) return;

    physics_character_virtual_build_global_triangle_mesh(geos.character_virtual_triangles);
    geos.character_virtual_mesh_built = character_virtual_global_mesh.valid;
    assert(geos.character_virtual_mesh_built, tprint("character virtual collision mesh failed to build: triangles=%", geos.character_virtual_triangles.count));
    write_character_virtual_collision_mesh_cache(geos, map_path, triangle_checksum);
    log("character virtual collision mesh %: triangles=% nodes=%",
        ifx geos.character_virtual_mesh_built then "built" else "failed",
        geos.character_virtual_triangles.count,
        character_virtual_global_mesh.node_count);
}

is_imported_map_trigger_name :: (name: string) -> bool {
    return begins_with(name, "Jump_Pad.{") ||
           begins_with(name, "Teleporter.{") ||
           begins_with(name, "King_Of_The_Hill_Point.{") ||
           begins_with(name, "Capture_The_Flag_Team_A_Flag_Zone.{") ||
           begins_with(name, "Capture_The_Flag_Team_B_Flag_Zone.{");
}

convert_obj_to_indexed_triangle_positions :: (obj_data: *Obj_Data) -> [..] Indexed_Triangle_Positions {
    results: [..] Indexed_Triangle_Positions;
    
    for *obj_object : obj_data.all_objects {
        if is_imported_map_trigger_name(obj_object.name) continue;

        itp: Indexed_Triangle_Positions;
        current_index: u32 = 0;
        
        for *face : obj_object.faces {
            if face.face_vertices.count < 3 continue;
            
            first_index := current_index;
            
            for *fv : face.face_vertices {
                array_add(*itp.positions, obj_data.all_positions[fv.position_index]);
                current_index += 1;
            }
            
            append_triangulated_polygon_indices(*itp.indices, itp.positions, first_index, face.face_vertices.count);
        }
        
        array_add(*results, itp);
    }
    
    return results;
}

load_in_collision_geometry_from_obj :: (map : *Collision_Geometries, map_obj_path : string) {

    obj_data := load_obj(map_obj_path);
    itps := convert_obj_to_indexed_triangle_positions(*obj_data);

    for itps {
        array_add(*map.aabbs, create_aabb_from_positions(it.positions));
        array_add(*map.indexed_triangle_positions, it);
    }

    array_free(itps);

    deinit(*obj_data);
    build_character_virtual_collision_mesh(map, map_obj_path);
}

gltf_node_local_transform :: (node: *Gltf_Node) -> Matrix4 {
    if node.has_matrix return node.matrix;

    rotation := rotation_matrix(Matrix4, node.rotation);

    scale_matrix := Matrix4_Identity;
    scale_matrix.coef[0][0] = node.scale.x;
    scale_matrix.coef[1][1] = node.scale.y;
    scale_matrix.coef[2][2] = node.scale.z;

    translation_matrix := Matrix4_Identity;
    translation_matrix.coef[0][3] = node.translation.x;
    translation_matrix.coef[1][3] = node.translation.y;
    translation_matrix.coef[2][3] = node.translation.z;

    return translation_matrix * rotation * scale_matrix;
}

transform_gltf_point :: (m: Matrix4, p: Vector3) -> Vector3 {
    return .{
        m.coef[0][0] * p.x + m.coef[0][1] * p.y + m.coef[0][2] * p.z + m.coef[0][3],
        m.coef[1][0] * p.x + m.coef[1][1] * p.y + m.coef[1][2] * p.z + m.coef[1][3],
        m.coef[2][0] * p.x + m.coef[2][1] * p.y + m.coef[2][2] * p.z + m.coef[2][3],
    };
}

append_gltf_primitive_to_itp :: (itp: *Indexed_Triangle_Positions, gltf: *Gltf_Data, primitive: *Gltf_Primitive, transform: Matrix4) {
    if primitive.mode != .TRIANGLES return;
    if primitive.position_accessor < 0 return;

    positions := read_accessor_as_vector3_array(gltf, primitive.position_accessor);
    defer array_free(positions);

    base_index := cast(u32) itp.positions.count;
    for positions array_add(*itp.positions, transform_gltf_point(transform, it));

    if primitive.indices_accessor >= 0 {
        indices := read_accessor_as_index_array(gltf, primitive.indices_accessor);
        defer array_free(indices);

        for indices array_add(*itp.indices, base_index + it);
    } else {
        for i: 0..positions.count - 1 array_add(*itp.indices, base_index + cast(u32) i);
    }
}

gltf_node_to_indexed_triangle_positions :: (gltf: *Gltf_Data, node: *Gltf_Node, transform: Matrix4) -> Indexed_Triangle_Positions {
    result: Indexed_Triangle_Positions;
    if node.mesh_index < 0 || node.mesh_index >= gltf.all_meshes.count return result;

    mesh := *gltf.all_meshes[node.mesh_index];
    for * primitive: mesh.primitives {
        append_gltf_primitive_to_itp(*result, gltf, primitive, transform);
    }

    return result;
}

load_in_collision_geometry_from_gltf_node_recursive :: (map : *Collision_Geometries, gltf: *Gltf_Data, node_index: int, parent_transform: Matrix4) {
    if node_index < 0 || node_index >= gltf.all_nodes.count return;

    node := *gltf.all_nodes[node_index];
    transform := parent_transform * gltf_node_local_transform(node);

    if !is_imported_map_trigger_name(node.name) {
        itp := gltf_node_to_indexed_triangle_positions(gltf, node, transform);
        if itp.positions.count > 0 && itp.indices.count > 0 {
            array_add(*map.aabbs, create_aabb_from_positions(itp.positions));
            array_add(*map.indexed_triangle_positions, itp);
        } else {
            deinit(*itp);
        }
    }

    for node.children {
        load_in_collision_geometry_from_gltf_node_recursive(map, gltf, it, transform);
    }
}

load_in_collision_geometry_from_gltf :: (map : *Collision_Geometries, map_gltf_path : string) {
    gltf := load_gltf(map_gltf_path);
    defer deinit(*gltf);

    scene := get_default_scene(*gltf);
    if scene {
        for scene.nodes {
            load_in_collision_geometry_from_gltf_node_recursive(map, *gltf, it, Matrix4_Identity);
        }
    } else {
        for i: 0..gltf.all_nodes.count - 1 {
            load_in_collision_geometry_from_gltf_node_recursive(map, *gltf, i, Matrix4_Identity);
        }
    }
    build_character_virtual_collision_mesh(map, map_gltf_path);
}

load_in_collision_geometry_from_model :: (map : *Collision_Geometries, map_path : string) {
    if ends_with(map_path, ".gltf") {
        load_in_collision_geometry_from_gltf(map, map_path);
    } else {
        load_in_collision_geometry_from_obj(map, map_path);
    }
}

Collision_Result :: struct {
    occurred := false;
    face_normal_generating_least_overlap : Vector3;
    overlap_along_normal : float;
    normal_is_pointing_towards_collision: bool;
}

compute_normal_of_triangle :: (a: Vector3, b: Vector3, c: Vector3) -> Vector3  {
    return normalized_cross(b - a, c - a);
}


// uses the separating axis theorem, worst case runtime is n * m where n and m are the number of faces on each shape respectively.
// this makes it cheap on simple objects, but not for complex ones.

detect_convex_collision :: (obj_1: Indexed_Triangle_Positions, obj_1_origin: Vector3, obj_2: Indexed_Triangle_Positions, obj_2_origin: Vector3) -> Collision_Result {

    collision_result : Collision_Result;

    min_overlap_so_far : float = FLOAT32_MAX;
    face_normal_generating_least_overlap : Vector3;

    // note that it's possible for this to be false, because if you have an axis aligned cube
    // then it's possible that the first face creates the min, and it could have been the paralell face that's pointing away
    normal_is_pointing_towards_collision := false;

    // note this is pretty wasteful, but I'm just focusing on making the algorithm work first.
    // we could compute as we go so that we don't waste.
    
    face_normals_on_all_objects : [..] Vector3;
    defer array_free(face_normals_on_all_objects);

    i := 0;
    while i + 2 < obj_1.indices.count {

        normal := compute_normal_of_triangle(
            obj_1.positions[obj_1.indices[i]], 
            obj_1.positions[obj_1.indices[i + 1]], 
            obj_1.positions[obj_1.indices[i + 2]], 
        );


        array_add(*face_normals_on_all_objects, normal);
        i += 3;
    }

    i = 0;
    while i + 2 < obj_2.indices.count {

        normal := compute_normal_of_triangle(
            obj_2.positions[obj_2.indices[i]], 
            obj_2.positions[obj_2.indices[i + 1]], 
            obj_2.positions[obj_2.indices[i + 2]], 
        );


        array_add(*face_normals_on_all_objects, normal);
        i += 3;
    }

    for face_normal : face_normals_on_all_objects {

        obj_1_min_projection_on_line_defined_by_face_normal := FLOAT32_MAX;
        obj_1_max_projection_on_line_defined_by_face_normal := -FLOAT32_MAX;

        for v : obj_1.positions {
            // recall that the dot product is ||a|| * ||b|| cos(theta)
            // where theta is the angle between the vectors a and b, if we suppose that b is a normal vector then
            // the equation becomes
            // ||a|| cos(theta), Since we know that cos(theta) = adj/hyp, and if we drop a perpendicular from a onto
            // the ray generated by b, then we can deduce that adj of that triangle equals hyp * cos(theta) and here
            // we know that ||a|| = hyp, therefore dot(v, face_normal) is the projection of the vertex v along the
            // normal vector's generated line

            projection_along_line_generated_by_face_normal : float = dot(v + obj_1_origin, face_normal);
            obj_1_min_projection_on_line_defined_by_face_normal = min(obj_1_min_projection_on_line_defined_by_face_normal, projection_along_line_generated_by_face_normal);
            obj_1_max_projection_on_line_defined_by_face_normal = max(obj_1_max_projection_on_line_defined_by_face_normal, projection_along_line_generated_by_face_normal);
        }

        obj_2_min_projection_on_line_defined_by_face_normal := FLOAT32_MAX;
        obj_2_max_projection_on_line_defined_by_face_normal := -FLOAT32_MAX;

        for v : obj_2.positions {
            projection_along_line_generated_by_face_normal : float = dot(v + obj_2_origin, face_normal);
            obj_2_min_projection_on_line_defined_by_face_normal = min(obj_2_min_projection_on_line_defined_by_face_normal, projection_along_line_generated_by_face_normal);
            obj_2_max_projection_on_line_defined_by_face_normal = max(obj_2_max_projection_on_line_defined_by_face_normal, projection_along_line_generated_by_face_normal);
        }

        overlap := min(obj_1_max_projection_on_line_defined_by_face_normal, obj_2_max_projection_on_line_defined_by_face_normal) -
                   max(obj_1_min_projection_on_line_defined_by_face_normal, obj_2_min_projection_on_line_defined_by_face_normal);

        if (overlap < 0) {
            // separating axis found, no collision (by the separating axis theorem)
            return collision_result;
        } else {
            // overlap was found on this line, but that doesn't mean they collide, they only collide if this occurs on all face normal lines.
        }

        if (overlap < min_overlap_so_far) {
            min_overlap_so_far = overlap;
            face_normal_generating_least_overlap = face_normal;

            // this measurement tells us this information because if you have two vectors a, b in space, and suppose
            // against some plane defined by v (unit vector) such that the projection of a onto v is 3 and the
            // projection of b on to v is 4, then you'll find the that projection of a onto -v is -3 and the projection
            // of b onto -v is -4. Note that 3 < 4 and then -3 > -4, thus this is why it works.
            normal_is_pointing_towards_collision = obj_1_max_projection_on_line_defined_by_face_normal > obj_2_max_projection_on_line_defined_by_face_normal;
        }
    }

    // note: if you get to this line in the code we know that the a collision has occurred, because no early return has been hit.
    
    collision_result.occurred = true;
    collision_result.face_normal_generating_least_overlap = face_normal_generating_least_overlap;

    collision_result.overlap_along_normal = min_overlap_so_far;
    collision_result.normal_is_pointing_towards_collision = normal_is_pointing_towards_collision;

    return collision_result;
}

// assumes that all objects are convex!
resolve_collisions_against_body :: (positioned_colliders : Collision_Geometries, body_collider: Collision_Geometry, body_position : *Vector3, body_velocity : *Vector3, on_ground: *bool) {

    on_ground.* = false;
    positioned_body_aabb := change_aabb_origin(body_collider.aabb, body_position);

    // eventually could be automated with a proper dynamic soa array.
    for i : 0..positioned_colliders.aabbs.count-1 {
        collider := Collision_Geometry.{positioned_colliders.indexed_triangle_positions[i], positioned_colliders.aabbs[i]};
        if aabbs_are_disjoint(collider.aabb, positioned_body_aabb) {
            continue;
        } 

        collision_result := detect_convex_collision(body_collider.indexed_triangle_positions, body_position, collider.indexed_triangle_positions, ZERO3);
        
        if (collision_result.occurred) {
            normal_facing_towards_body := ifx collision_result.normal_is_pointing_towards_collision then collision_result.face_normal_generating_least_overlap else -collision_result.face_normal_generating_least_overlap;

            pos_offset_to_escape_collision := normal_facing_towards_body * collision_result.overlap_along_normal;
            body_position.* += pos_offset_to_escape_collision;

            projected_velocity_length_along_normal := dot(body_velocity, normal_facing_towards_body);
            velocity_is_going_opposite_direction_of_push := projected_velocity_length_along_normal < 0.0;

            if (velocity_is_going_opposite_direction_of_push)
                body_velocity.* -= projected_velocity_length_along_normal * normal_facing_towards_body;

            up_alignment := dot(normal_facing_towards_body, Y3);
            cos_45 := 0.707;

            if (up_alignment > cos_45) {
                on_ground.* = true;
            }
        } // otherwise aabbs collided but the inner geometry did not.
    }
}

Ray_Hit_Result :: struct {
    hit := false;
    hit_percent : float;  
    hit_point : Vector3;
    hit_normal : Vector3;
    triangle_index : int = -1;  // which triangle was hit
}

// Möller–Trumbore ray-triangle intersection algorithm: https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
intersect_ray_triangle :: (ray: Ray, v0: Vector3, v1: Vector3, v2: Vector3) -> has_intersection: bool, t: float, u: float, v: float {
    EPSILON :: 0.000001;
    
    edge1 := v1 - v0;
    edge2 := v2 - v0;
    
    h := cross(ray.reach, edge2);
    a := dot(edge1, h);
    
    ray_is_parallel_to_triangle := a > -EPSILON && a < EPSILON;
    if ray_is_parallel_to_triangle return false, 0, 0, 0;
    
    f := 1.0 / a;
    s := ray.origin - v0;
    u := f * dot(s, h);
    
    u_outside_triangle := u < 0.0 || u > 1.0;
    if u_outside_triangle return false, 0, 0, 0;
    
    q := cross(s, edge1);
    v := f * dot(ray.reach, q);
    
    v_outside_triangle := v < 0.0 || u + v > 1.0;
    if v_outside_triangle return false, 0, 0, 0;
    
    t := f * dot(edge2, q);
    
    hit_at_valid_place_on_ray := t > EPSILON && t <= 1.0;
    if hit_at_valid_place_on_ray return true, t, u, v;
    
    return false, 0, 0, 0;
}

// assumes geometry vertices are already in world space
intersect_ray_geometry :: (ray: Ray, itp: Indexed_Triangle_Positions) -> Ray_Hit_Result {
    result : Ray_Hit_Result;
    closest_t := FLOAT32_MAX;
    
    i := 0;
    triangle_index := 0;
    while i + 2 < itp.indices.count {
        v0 := itp.positions[itp.indices[i]];
        v1 := itp.positions[itp.indices[i + 1]];
        v2 := itp.positions[itp.indices[i + 2]];
        
        has_hit, t, u, v := intersect_ray_triangle(ray, v0, v1, v2);
        
        if has_hit && t < closest_t && t >= 0 {
            closest_t = t;
            result.hit = true;
            result.hit_percent = t;
            result.hit_point = ray.origin + ray.reach * t;
            result.hit_normal = compute_normal_of_triangle(v0, v1, v2);
            result.triangle_index = triangle_index;
        }
        
        i += 3;
        triangle_index += 1;
    }
    
    return result;
}

// everything is in world space.
can_hit_target_geometry :: ( ray: Ray, target: Collision_Geometry, blockers: Collision_Geometries) -> hit: bool, hit_result: Ray_Hit_Result {

    target_aabb_hit, target_aabb_percent := intersect_ray(ray, target.aabb);
    if !target_aabb_hit return false, .{}; // we we can't even reach the aabb of the target, then go away.
    
    target_hit_result := intersect_ray_geometry(ray, target.indexed_triangle_positions);
    if !target_hit_result.hit return false, .{}; // if we can't even hit the target iself, then go away.
    
    // now it's guarenteed that the ray intersects the target.

    for i : 0..blockers.aabbs.count-1 {
        blocker_aabb_hit, blocker_aabb_percent := intersect_ray(ray, blockers.aabbs[i]);
        
        if !blocker_aabb_hit continue;  // if you don't hit the aabb, you don't hit what's inside.

        if blocker_aabb_percent >= target_hit_result.hit_percent continue; // the hit is further away than the hit on the targets aabb, not blocking so we can skip narrowphase

        blocker_hit_result := intersect_ray_geometry(ray, blockers.indexed_triangle_positions[i]);
        
        if blocker_hit_result.hit && blocker_hit_result.hit_percent < target_hit_result.hit_percent return false, .{};  // the ray was blocked by this geometry.
    }
    
    return true, target_hit_result;
}
