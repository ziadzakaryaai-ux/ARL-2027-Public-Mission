def monitor_following_distance(distances: list[float], speeds: list[float]) -> tuple[int, float, int]:
    """
    Analyzes following distance compared to safe distance (speed * 0.5).
    
    Args:
        distances (list[float]): Distance to the lead car at each second.
        speeds (list[float]): Speed of our car at each second.
        
    Returns:
        tuple[int, float, int]: (tailgating_seconds, minimum_distance, tailgate_incidents)
            - tailgating_seconds: total seconds distance was < safe distance
            - minimum_distance: absolute closest distance to the lead car (return 0.0 if empty list)
            - tailgate_incidents: number of separate instances the car started tailgating
    """
    pass

    if not distances and not speeds:
        return 0, 0.0, 0

    if len(distances) != len(speeds):
        raise ValueError(f"Input list length mismatch: distances ({len(distances)}) vs speeds ({len(speeds)}).")

    tailgating_seconds = 0
    minimum_distance = float('inf')
    tailgate_incidents = 0
    in_tailgate = False

    for dist, speed in zip(distances, speeds):
        if dist < minimum_distance:
            minimum_distance = dist

        safe_distance = speed * 0.5

        if dist < safe_distance:
            tailgating_seconds += 1
            if not in_tailgate:
                tailgate_incidents += 1
                in_tailgate = True
        else:
            in_tailgate = False

    return tailgating_seconds, minimum_distance,tailgate_incidents
