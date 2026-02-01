#import "Math";

#import "tbx/math";

AABB :: Axis_Aligned_Bounding_Box;

Axis_Aligned_Bounding_Box :: struct {
    min, max : Vector3; 
}

create_aabb_from_positions :: (positions: [..] Vector3) -> AABB {
    min_so_far := ONE3 * FLOAT32_MAX;
    max_so_far := ONE3 * FLOAT32_MIN;

    for positions {
        min_so_far = min(min_so_far, it);
        max_so_far = max(max_so_far, it);
    }

    return .{min_so_far, max_so_far};
}

change_aabb_origin :: (aabb : AABB, position: Vector3) -> AABB {
    out : AABB;

    half_extents := (aabb.max - aabb.min) * 0.5;

    out.min = position - half_extents;
    out.max = position + half_extents;

    return out;
}

aabbs_are_disjoint :: (a : AABB, b : AABB) -> bool {
return (a.max.x <= b.min.x || a.min.x >= b.max.x || 
        a.max.y <= b.min.y || a.min.y >= b.max.y ||
        a.max.z <= b.min.z || a.min.z >= b.max.z);
}

aabbs_overlap :: (a : AABB, b : AABB) -> bool {
    return !aabbs_are_disjoint(a, b);
}
