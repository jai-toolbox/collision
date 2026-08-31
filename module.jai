// in the future I don't want to depend on all of geom, because all I need is the itp type doing it for now.
#import "tbx/geometry";
#import "tbx/model_loading";
#import "tbx/physics";

#load "axis_aligned_bounding_box.jai";

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
    // Imported trigger meshes that should be drawn but must not participate in
    // movement, ray tests, navigation, or the CharacterVirtual collision mesh.
    // Jump pads and teleporters use their authored trigger mesh as their
    // visible surface.
    visual_only_indexed_triangle_positions : [..] Indexed_Triangle_Positions;
    aabbs : [..] AABB;
    kinematic_grid: Kinematic_Collision_Grid;
    character_virtual_triangles: [..] Geometry_Triangle;
    character_virtual_mesh_built := false;
}

deinit :: (using geos: *Collision_Geometries) {
    for *indexed_triangle_positions deinit(it);
    array_free(indexed_triangle_positions);
    for *visual_only_indexed_triangle_positions deinit(it);
    array_free(visual_only_indexed_triangle_positions);
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

is_visible_imported_map_trigger_name :: (name: string) -> bool {
    return begins_with(name, "Jump_Pad.{") ||
           begins_with(name, "Teleporter.{");
}

obj_object_to_indexed_triangle_positions :: (obj_data: *Obj_Data, obj_object: *Obj_Object) -> Indexed_Triangle_Positions {
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

    return itp;
}

convert_obj_to_indexed_triangle_positions :: (obj_data: *Obj_Data) -> [..] Indexed_Triangle_Positions {
    results: [..] Indexed_Triangle_Positions;
    
    for *obj_object : obj_data.all_objects {
        if is_imported_map_trigger_name(obj_object.name) continue;

        itp := obj_object_to_indexed_triangle_positions(obj_data, obj_object);
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

    for *obj_object : obj_data.all_objects {
        if !is_visible_imported_map_trigger_name(obj_object.name) continue;

        itp := obj_object_to_indexed_triangle_positions(*obj_data, obj_object);
        if itp.positions.count > 0 && itp.indices.count > 0 {
            array_add(*map.visual_only_indexed_triangle_positions, itp);
        } else {
            deinit(*itp);
        }
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

    is_trigger := is_imported_map_trigger_name(node.name);
    is_visible_trigger := is_visible_imported_map_trigger_name(node.name);
    if !is_trigger || is_visible_trigger {
        itp := gltf_node_to_indexed_triangle_positions(gltf, node, transform);
        if itp.positions.count > 0 && itp.indices.count > 0 {
            if is_visible_trigger {
                array_add(*map.visual_only_indexed_triangle_positions, itp);
            } else {
                array_add(*map.aabbs, create_aabb_from_positions(itp.positions));
                array_add(*map.indexed_triangle_positions, itp);
            }
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

Ray_Hit_Result :: struct {
    hit := false;
    hit_percent : float;
    hit_point : Vector3;
    hit_normal : Vector3;
    triangle_index : int = -1;  // which triangle was hit
}

physics_segment_raycast_aabb :: (ray: Ray, aabb: AABB) -> bool, float {
    // physics_ray_aabox operates on an unbounded ray. Here Ray.reach is the
    // complete segment, so t > 1 lies beyond its end. A negative entry means
    // the segment begins inside the box and is reported as a hit at t = 0.
    inverse_direction := physics_make_ray_inv_direction(ray.reach);
    fraction := physics_ray_aabox(ray.origin, inverse_direction, aabb.min, aabb.max);
    if fraction == FLOAT32_MAX || fraction > 1.0 return false, 0.0;
    return true, max(0.0, fraction);
}

// Imported/indexed geometry remains owned by this module. Gameplay query math
// passes through these adapters and is performed by the physics module.
physics_raycast_indexed_geometry :: (ray: Ray, itp: Indexed_Triangle_Positions, offset: Vector3 = .{}) -> Ray_Hit_Result {
    result: Ray_Hit_Result;
    result.hit_percent = FLOAT32_MAX;

    triangle_count := itp.indices.count / 3;
    for triangle_index: 0..triangle_count - 1 {
        v0 := itp.positions[itp.indices[triangle_index * 3 + 0]] + offset;
        v1 := itp.positions[itp.indices[triangle_index * 3 + 1]] + offset;
        v2 := itp.positions[itp.indices[triangle_index * 3 + 2]] + offset;
        fraction := physics_ray_triangle(ray.origin, ray.reach, v0, v1, v2);
        if fraction > 1.0 || fraction >= result.hit_percent continue;

        result.hit = true;
        result.hit_percent = fraction;
        result.hit_point = ray.origin + ray.reach * fraction;
        result.hit_normal = physics_normalized_or(cross(v1 - v0, v2 - v0), Y3);
        result.triangle_index = triangle_index;
    }

    return result;
}

physics_raycast_collision_map :: (ray: Ray, map: Collision_Geometries) -> Ray_Hit_Result {
    result: Ray_Hit_Result;
    result.hit_percent = FLOAT32_MAX;

    for i: 0..map.aabbs.count - 1 {
        aabb_hit, aabb_fraction := physics_segment_raycast_aabb(ray, map.aabbs[i]);
        if !aabb_hit || aabb_fraction >= result.hit_percent continue;

        hit := physics_raycast_indexed_geometry(ray, map.indexed_triangle_positions[i]);
        if hit.hit && hit.hit_percent < result.hit_percent result = hit;
    }

    return result;
}

physics_sphere_cast_indexed_geometry :: (start: Vector3, direction: Vector3, radius: float, itp: Indexed_Triangle_Positions, offset: Vector3 = .{}) -> Ray_Hit_Result {
    result: Ray_Hit_Result;
    result.hit_percent = FLOAT32_MAX;

    triangle_count := itp.indices.count / 3;
    for triangle_index: 0..triangle_count - 1 {
        triangle := Geometry_Triangle.{
            itp.positions[itp.indices[triangle_index * 3 + 0]] + offset,
            itp.positions[itp.indices[triangle_index * 3 + 1]] + offset,
            itp.positions[itp.indices[triangle_index * 3 + 2]] + offset,
        };
        hit := physics_cast_sphere_triangle(start, direction, radius, triangle);
        if !hit.hit || hit.fraction > 1.0 || hit.fraction >= result.hit_percent continue;

        result.hit = true;
        result.hit_percent = hit.fraction;
        result.hit_point = start + direction * hit.fraction;
        result.hit_normal = hit.normal;
        result.triangle_index = triangle_index;
    }

    return result;
}

physics_sphere_cast_collision_map :: (start: Vector3, direction: Vector3, radius: float, map: Collision_Geometries) -> Ray_Hit_Result {
    result: Ray_Hit_Result;
    result.hit_percent = FLOAT32_MAX;
    radius_vector := ONE3 * radius;

    for i: 0..map.aabbs.count - 1 {
        expanded := AABB.{map.aabbs[i].min - radius_vector, map.aabbs[i].max + radius_vector};
        aabb_hit, aabb_fraction := physics_segment_raycast_aabb(.{start, direction}, expanded);
        if !aabb_hit || aabb_fraction >= result.hit_percent continue;

        hit := physics_sphere_cast_indexed_geometry(start, direction, radius, map.indexed_triangle_positions[i]);
        if hit.hit && hit.hit_percent < result.hit_percent result = hit;
    }

    return result;
}

physics_point_inside_indexed_geometry :: (point: Vector3, itp: Indexed_Triangle_Positions, offset: Vector3 = .{}) -> bool {
    // Count unique crossings so both triangles of a quad count as one surface.
    // The skewed ray avoids common axis-aligned edge and vertex cases.
    direction := Vector3.{104729.0, 176081.0, 235111.0};
    crossing_count := 0;
    triangle_count := itp.indices.count / 3;

    for triangle_index: 0..triangle_count - 1 {
        v0 := itp.positions[itp.indices[triangle_index * 3 + 0]] + offset;
        v1 := itp.positions[itp.indices[triangle_index * 3 + 1]] + offset;
        v2 := itp.positions[itp.indices[triangle_index * 3 + 2]] + offset;
        fraction := physics_ray_triangle(point, direction, v0, v1, v2);
        if fraction == FLOAT32_MAX || fraction <= 0.000001 || fraction > 1.0 continue;

        duplicate := false;
        for previous_index: 0..triangle_index - 1 {
            p0 := itp.positions[itp.indices[previous_index * 3 + 0]] + offset;
            p1 := itp.positions[itp.indices[previous_index * 3 + 1]] + offset;
            p2 := itp.positions[itp.indices[previous_index * 3 + 2]] + offset;
            previous_fraction := physics_ray_triangle(point, direction, p0, p1, p2);
            if previous_fraction != FLOAT32_MAX && abs(previous_fraction - fraction) <= 0.000001 {
                duplicate = true;
                break;
            }
        }
        if !duplicate crossing_count += 1;
    }

    return crossing_count % 2 == 1;
}

physics_sphere_touches_indexed_geometry :: (center: Vector3, radius: float, itp: Indexed_Triangle_Positions, offset: Vector3 = .{}) -> bool {
    triangle_count := itp.indices.count / 3;
    for triangle_index: 0..triangle_count - 1 {
        triangle := Geometry_Triangle.{
            itp.positions[itp.indices[triangle_index * 3 + 0]] + offset,
            itp.positions[itp.indices[triangle_index * 3 + 1]] + offset,
            itp.positions[itp.indices[triangle_index * 3 + 2]] + offset,
        };
        if physics_sphere_triangle_contact(center, radius, triangle, 0.0).hit return true;
    }
    return physics_point_inside_indexed_geometry(center, itp, offset);
}

physics_character_capsule_touches_geometry :: (
    position: Vector3,
    geometry: Collision_Geometry,
    capsule_half_height: float = (1.75 - 2.0 * 0.4) * 0.5,
    capsule_radius: float = 0.4
) -> bool {
    body: Physics_Body;
    body.kind = .DYNAMIC;
    body.shape.kind = .CAPSULE;
    body.shape.radius = capsule_radius;
    body.shape.half_height_of_cylinder = capsule_half_height;
    body.position = position;

    itp := geometry.indexed_triangle_positions;
    triangle_count := itp.indices.count / 3;
    for triangle_index: 0..triangle_count - 1 {
        triangle := Geometry_Triangle.{
            itp.positions[itp.indices[triangle_index * 3 + 0]],
            itp.positions[itp.indices[triangle_index * 3 + 1]],
            itp.positions[itp.indices[triangle_index * 3 + 2]],
        };
        contact := physics_body_triangle_contact(*body, triangle, triangle_index, 0.0);
        if contact.active && contact.separation <= 0.0 return true;
    }

    // A capsule wholly inside a closed trigger has no surface contact.
    return physics_point_inside_indexed_geometry(position, itp);
}

physics_character_capsule_overlaps_aabb :: (
    position: Vector3,
    aabb: AABB,
    capsule_half_height: float = (1.75 - 2.0 * 0.4) * 0.5,
    capsule_radius: float = 0.4
) -> bool {
    segment_min_y := position.y - capsule_half_height;
    segment_max_y := position.y + capsule_half_height;

    dx := max(max(aabb.min.x - position.x, 0.0), position.x - aabb.max.x);
    dz := max(max(aabb.min.z - position.z, 0.0), position.z - aabb.max.z);
    dy: float;
    if segment_max_y < aabb.min.y dy = aabb.min.y - segment_max_y;
    if segment_min_y > aabb.max.y dy = segment_min_y - aabb.max.y;
    return dx * dx + dy * dy + dz * dz <= capsule_radius * capsule_radius;
}

physics_can_hit_target_geometry :: (ray: Ray, target: Collision_Geometry, blockers: Collision_Geometries) -> hit: bool, hit_result: Ray_Hit_Result {
    // The target AABB is a cheap broad-phase rejection. Only a possible AABB
    // hit proceeds to the exact triangle test, after which the closest exact
    // blocker hit decides whether the target is visible.
    target_aabb_hit, _ := physics_segment_raycast_aabb(ray, target.aabb);
    if !target_aabb_hit return false, .{};

    target_hit_result := physics_raycast_indexed_geometry(ray, target.indexed_triangle_positions);
    if !target_hit_result.hit return false, .{};

    blocker_hit := physics_raycast_collision_map(ray, blockers);
    if blocker_hit.hit && blocker_hit.hit_percent < target_hit_result.hit_percent return false, .{};
    return true, target_hit_result;
}
