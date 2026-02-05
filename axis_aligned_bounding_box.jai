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

Ray :: struct {
    // note we didn't call it direction because that usually implies that length doesn't matter.
    origin, reach : Vector3;
}

#import "Sloppy_Math";

// note we guarentee that first hit percent is non-negative for anything that has intersection.
intersect_ray :: (ray : Ray, aabb: AABB) -> has_intersection: bool, first_hit_percent : float {
    max_entry_point_percent : float = 0;
    min_exit_point_percent := FLOAT32_MAX;

    // we compute intersection by looking at it componentwise
    for axis : 0..3-1 {
        origin := ray.origin.component[axis];
        reach := ray.reach.component[axis];
        aabb_min := aabb.min.component[axis];
        aabb_max := aabb.max.component[axis];

        if is_approximately_zero(reach) {
            origin_outside_aabb := origin < aabb_min || aabb_max < origin;
            if origin_outside_aabb return false, 0;
        } else {
            // origin + t * reach = aabb_min <=> (aabb_min - origin) / reach = t
            // thus we can get back a t value and determine if it's valid (ie t in [0, 1])

            /*
                       aabb_min --------------- aabb_max
            (1)  r-----------------> 
            (2)                               <------------------r
            */

            // assumes case (1)
            aabb_entry := (aabb_min - origin) / reach;
            aabb_exit := (aabb_max - origin) / reach;

            if (aabb_entry > aabb_exit) { // (2) needs re-ordering since it hits max, and then min (entry and exit are wrong by default)
                aabb_entry, aabb_exit = aabb_exit, aabb_entry;
            }

            // to understand this it's best to think in 2d, axis-wise we can deduce the interval of intersection of our ray
            // suppose on the x axis the interval is I1, and on y it's I2, if I1 intersect I2 is non-empty then this ray goes inside the aabb.
            // concretely suppose x's t interval is [2, 5] and y's is [7, 10], so the ray hits those in two disjoint segments on the ray
            // and thus no intersection. In the algorithm our max_entry_point would go 0 -> 2 -> 7 and min_entry_point -> oo, 5, 5
            // without proving stuff you can see that the max entry point become greater than the min entry point, so no intersection
            // prove later if interested.

            max_entry_point_percent = max(max_entry_point_percent, aabb_entry);
            min_exit_point_percent = min(min_exit_point_percent , aabb_exit);

            if (max_entry_point_percent > min_exit_point_percent) return false, 0;
        }
    }

    // if you make it here it means that on each axis the entry and exit got closer but never crossed eachother, so there is intersection

    // note that returning the max makes sense, because if you did the say the min, then you're not guarenteeing that this entry point
    // hits on all axes
    return true, max_entry_point_percent;
}

// this runs for at most blockers.count, so could be expensive on big map, keep in mind
// potentially make more efficient with a bvh if this gets slow, should be fine until more players are playin.
can_hit_target_aabb :: (ray : Ray, target : AABB, blockers : [] AABB) -> bool {
    target_has_intersection, target_first_hit_percent := intersect_ray(ray, target);
    if !target_has_intersection return false; // you couldn't even reach it, go away.

    for blockers {
        blocker_has_intersection, blocker_first_hit_percent := intersect_ray(ray, it);
        if !blocker_has_intersection continue;
        if blocker_first_hit_percent < target_first_hit_percent return false;
    }

    return true; // we can hit it if nothing is blocking it.
}
