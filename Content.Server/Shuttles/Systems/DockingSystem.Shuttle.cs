using System.Linq;
using System.Numerics;
using Content.Server.Shuttles.Components;
using Content.Shared._Triad.Holosign;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Collision.Shapes;
using Robust.Shared.Physics.Components;
using Content.Shared.Shuttles.Components; // Frontier

namespace Content.Server.Shuttles.Systems;

public sealed partial class DockingSystem
{
    /*
     * Handles the shuttle side of FTL docking.
     */

    private const int DockRoundingDigits = 2;

    public Angle GetAngle(EntityUid uid, TransformComponent xform, EntityUid targetUid, TransformComponent targetXform)
    {
        var (shuttlePos, shuttleRot) = _transform.GetWorldPositionRotation(xform);
        var (targetPos, targetRot) = _transform.GetWorldPositionRotation(targetXform);

        var shuttleCOM = Robust.Shared.Physics.Transform.Mul(new Transform(shuttlePos, shuttleRot),
            _physicsQuery.GetComponent(uid).LocalCenter);
        var targetCOM = Robust.Shared.Physics.Transform.Mul(new Transform(targetPos, targetRot),
            _physicsQuery.GetComponent(targetUid).LocalCenter);

        var mapDiff = shuttleCOM - targetCOM;
        var angle = mapDiff.ToWorldAngle();
        angle -= targetRot;
        return angle;
    }

    /// <summary>
    /// Tries to get a valid docking configuration for the shuttle to the target grid.
    /// </summary>
    /// <param name="priorityTag">Priority docking tag to prefer, e.g. for emergency shuttle</param>
    public DockingConfig? GetDockingConfig(
        EntityUid shuttleUid,
        EntityUid stationUid,
        string? priorityTag = null,
        DockType dockType = DockType.Airlock)// Frontier: add
    {
        var targetGridAngle = _transform.GetWorldRotation(stationUid).Reduced();

        var conf = GetDockingConfigFast(shuttleUid,
            stationUid,
            dc =>
            {
                var configPriority = !IsConfigPriority(dc, priorityTag);
                var dockCount = dc.Docks.Count;
                var angleDelta = Math.Abs(Angle.ShortestDistance(dc.Angle.Reduced(), targetGridAngle).Theta);
                return (!configPriority, -dockCount, angleDelta); // It tries to minimize from left to right, so lower is better
            },
            Comparer<(bool, int, double)>.Default,
            dockType
        );

        if (conf is not null)
        {
            conf.TargetGridUid = stationUid;
        }

        return conf;
    }
    public DockingConfig? GetDockingConfigAt(
        EntityUid shuttleUid,
        EntityUid stationUid,
        EntityCoordinates coordinates,
        Angle angle,
        bool fallback = true,
        DockType dockType = DockType.Airlock, // Frontier
        string? priorityTag = null) // Triad: prefer a berth flagged for this traffic, see the fallback
    {
        var conf = GetDockingConfigFast(shuttleUid,
            stationUid,
            dc =>
            {
                var matchesPosition = dc.Coordinates.Equals(coordinates) && dc.Angle.EqualsApprox(angle, 0.15);
                var configPriority = !IsConfigPriority(dc, priorityTag);
                var dockCount = -dc.Docks.Count;
                var angleDelta = Math.Abs(Angle.ShortestDistance(dc.Angle.Reduced(), angle).Theta);
                return (!matchesPosition, !configPriority, -dockCount, angleDelta); // It tries to minimize from left to right, so lower is better
            },
            Comparer<(bool, bool, int, double)>.Default,
            dockType);

        // Triad: if the exact requested berth was not free, so rank the rest rather than grabbing
        // whatever enumerated first out of a HashSet. Same comparison departure used, so the
        // fallback lands on the berth and the approach side that were planned for. With no tag this
        // is the old behaviour, since neither tag predicate matches null.
        if (conf is not null && fallback == false)
        {
            if (conf.Coordinates.Equals(coordinates) && conf.Angle.EqualsApprox(angle, 0.15))
            {
                return conf;
            }
            else
            {
                return null;
            }
        }
        else
        {
            return conf;
        }
    }

    public bool IsConfigPriority(DockingConfig config, string? priorityTag)
    {
        return config.Docks.Any(docks =>
            TryComp<PriorityDockComponent>(docks.StationDockUid, out var priority)
            && priority.Tag?.Equals(priorityTag) == true);
    }

    /// <summary>
    /// Whether this shuttle port satisfies the station dock's side requirement, if it declares one.
    /// </summary>
    // Triad: hard filter, not a preference. A berth with RequiredShuttleTag set never pairs with an
    // untagged or wrong-side port, so the shuttle presents the declared side or the config never forms
    // and the trip is refused upstream. Berths with no requirement behave exactly as before.
    private bool ShuttleSideAllowed(EntityUid shuttleDockUid, EntityUid gridDockUid)
    {
        if (!TryComp<PriorityDockComponent>(gridDockUid, out var gridPriority)
            || gridPriority.RequiredShuttleTag is not { } required)
        {
            return true;
        }

        return TryComp<PriorityDockComponent>(shuttleDockUid, out var shuttlePriority)
               && shuttlePriority.Tag?.Equals(required) == true;
    }

    /// <summary>
    /// Checks whether the shuttle can warp to the specified position.
    /// </summary>
    private bool ValidSpawn(Entity<MapGridComponent> gridEntity, Matrix3x2 matty, Angle angle, FixturesComponent shuttleFixturesComp, bool isMap)
    {
        var transform = new Transform(Vector2.Transform(Vector2.Zero, matty), angle);

        // Because some docking bounds are tight af need to check each chunk individually
        foreach (var fix in shuttleFixturesComp.Fixtures.Values)
        {
            var polyShape = (PolygonShape)fix.Shape;
            var aabb = polyShape.ComputeAABB(transform, 0);
            aabb = aabb.Enlarged(-0.01f);

            // If it's a map check no hard collidable anchored entities overlap
            if (isMap)
            {
                var localTiles = _mapSystem.GetLocalTilesIntersecting(gridEntity.Owner, gridEntity.Comp, aabb);

                while (localTiles.MoveNext(out var tile))
                {
                    var anchoredEnumerator = _mapSystem.GetAnchoredEntities(gridEntity.Owner, gridEntity.Comp, tile.GridIndices);

                    while (anchoredEnumerator.MoveNext(out var anc))
                    {
                        if (!_physicsQuery.TryGetComponent(anc, out var physics) ||
                            !physics.CanCollide ||
                            !physics.Hard)
                        {
                            continue;
                        }

                        return false;
                    }
                }
            }
            // If it's not a map then we're docking onto another grid; ensure we don't overlap either tiles OR
            // anchored hard-collidable entities on that grid. Previously this only checked tiles which could
            // miss walls/fixtures and allow clipping.
            else
            {
                // First reject if any solid tiles intersect.
                if (_mapSystem.GetLocalTilesIntersecting(gridEntity.Owner, gridEntity.Comp, aabb).Any())
                    return false;

                // Additionally scan anchored entities on intersecting tiles and reject if any hard colliders are present.
                var localTiles = _mapSystem.GetLocalTilesIntersecting(gridEntity.Owner, gridEntity.Comp, aabb);
                while (localTiles.MoveNext(out var tile))
                {
                    var anchoredEnumerator = _mapSystem.GetAnchoredEntities(gridEntity.Owner, gridEntity.Comp, tile.GridIndices);
                    while (anchoredEnumerator.MoveNext(out var anc))
                    {
                        if (!_physicsQuery.TryGetComponent(anc, out var physics) ||
                            !physics.CanCollide ||
                            !physics.Hard)
                        {
                            continue;
                        }

                        return false;
                    }
                }
            }
        }

        return true;
    }

    public List<Entity<DockingComponent>> GetDocks(EntityUid uid)
    {
        _dockingSet.Clear();
        _lookup.GetChildEntities(uid, _dockingSet);

        return _dockingSet.ToList();
    }

    /// <summary>
    /// Mono: Checks if two grids are docked together by examining if any docking port on gridA is connected to any docking port on gridB.
    /// </summary>
    public bool AreGridsDocked(EntityUid gridA, EntityUid gridB)
    {
        var docksA = GetDocks(gridA);

        foreach (var dockA in docksA)
        {
            if (!dockA.Comp.Docked || dockA.Comp.DockedWith == null)
                continue;

            // Get the grid that this dock is connected to
            var connectedDockGrid = Transform(dockA.Comp.DockedWith.Value).GridUid;
            if (connectedDockGrid == gridB)
                return true;
        }

        return false;
    }


    public DockingConfig? GetDockingConfig(
        EntityUid shuttleUid,
        EntityUid stationUid,
        EntityUid shuttleDockUid,
        EntityUid stationDockUid)
    {

        var shuttleDockXform = _xformQuery.GetComponent(shuttleDockUid);
        var shuttleDockPosition = shuttleDockXform.LocalPosition;
        var stationDockXform = _xformQuery.GetComponent(stationDockUid);
        var stationDockPosition = stationDockXform.LocalPosition;

        var conf = GetDockingConfigFast(shuttleUid,
            stationUid,
            (dc) =>
            {
                var hasPair = false;
                var hasShuttle = false;
                var hasStation = false;
                foreach (var dockPair in dc.Docks)
                {
                    if (dockPair.ShuttleDockUid == shuttleDockUid)
                    {
                        hasShuttle = true;
                        if (dockPair.StationDockUid == stationDockUid)
                        {
                            hasStation = true;
                            hasPair = true;
                        }
                    }
                    else
                    {
                        if (dockPair.StationDockUid == stationDockUid)
                        {
                            hasStation = true;
                        }
                    }
                }
                var hasBoth = hasShuttle && hasStation;
                var squaredDistance = (Vector2.Transform(shuttleDockPosition, dc.Matty) - stationDockPosition)
                    .LengthSquared();
                return (!hasPair, !hasBoth, squaredDistance); // It tries to minimize from left to right, so lower is better
            },
            Comparer<(bool, bool, double)>.Default
        );

        if (conf is not null)
        {
            conf.TargetGridUid = stationUid;
        }

        return conf;
    }


    private DockingConfig? GetDockingConfigFast<TPriority>(
        EntityUid shuttleUid,
        EntityUid stationUid,
        Func<DockingConfig, TPriority> transformer,
        Comparer<TPriority> comparer,
        DockType dockType = DockType.Airlock)
    {
        var stationDocks = GetDocks(stationUid);
        var shuttleDocks = GetDocks(shuttleUid);

        var stationGrid = _gridQuery.GetComponent(shuttleUid);
        var shuttleGrid = _gridQuery.GetComponent(shuttleUid);
        var shuttleLocalAABB = shuttleGrid.LocalAABB;
        var shuttleFixturesComp = Comp<FixturesComponent>(shuttleUid);
        var stationIsMap = HasComp<MapComponent>(shuttleUid);
        var stationXform = _xformQuery.GetComponent(stationUid);

        var stationWorldAngle = _transform.GetWorldRotation(stationXform).Reduced();

        var configs = GetCandidateDockingConfigs(
            stationUid,
            shuttleDocks,
            stationDocks,
            shuttleGrid.LocalAABB,
            dockType,
            comparer,
            transformer
        );

        List<Entity<MapGridComponent>> grids = [];
        while (configs.Count != 0)
        {
            var dc = configs.Dequeue();
            if (DockingConfigCandidateIsValid(dc, stationUid, stationXform, shuttleLocalAABB, shuttleFixturesComp, stationGrid, stationIsMap, stationWorldAngle, ref grids))
            {
                return dc;
            }

        }

        return null;
    }

    private bool DockingConfigCandidateIsValid(
        DockingConfig dc,
        EntityUid stationUid,
        TransformComponent stationXform,
        Box2 shuttleLocalAABB,
        FixturesComponent shuttleFixturesComp,
        MapGridComponent stationGrid,
        bool stationIsMap,
        Angle stationWorldAngle,
        ref List<Entity<MapGridComponent>> grids)
    {
        if (!dc.Docks.Any())
            return false;

        var firstDockPair = dc.Docks.First();

        if (firstDockPair.ShuttleDock.Docked ||
            firstDockPair.StationDock.Docked ||
            !firstDockPair.ShuttleDockXform.Anchored ||
            !firstDockPair.StationDockXform.Anchored)
        {
            return false;
        }

        var trueOffsetAngle = (firstDockPair.StationDockXform.LocalRotation.Opposite() - firstDockPair.ShuttleDockXform.LocalRotation).Reduced();
        dc.SetMatty(firstDockPair.ShuttleDockXform, firstDockPair.StationDockXform);
        var gridEntity = new Entity<MapGridComponent>(stationUid, stationGrid);
        if (!ValidSpawn(gridEntity, dc.Matty, trueOffsetAngle, shuttleFixturesComp, stationIsMap))
        {
            return false;
        }

        // Can't just use the AABB as we want to get bounds as tight as possible.
        var gridPosition = new EntityCoordinates(stationUid, Vector2.Transform(Vector2.Zero, dc.Matty));
        var spawnPosition = new EntityCoordinates(stationXform.MapUid!.Value, _transform.ToMapCoordinates(gridPosition).Position);

        // TODO: use tight bounds
        var targetWorldAngle = (stationWorldAngle + trueOffsetAngle).Reduced(); // Frontier
        var dockedBounds = new Box2Rotated(shuttleLocalAABB.Translated(spawnPosition.Position), targetWorldAngle, spawnPosition.Position); // Frontier: targetAngle<targetWorldAngle

        // Check if there's no intersecting grids (AKA oh god it's docking at cargo).
        grids.Clear();
        _mapSystem.FindGridsIntersecting(stationXform.MapID, dockedBounds, ref grids, includeMap: false);
        if (grids.Any(o => o.Owner != dc.TargetGridUid && o.Owner != stationXform.MapUid))
        {
            return false;
        }

        return true;
    }


    private PriorityQueue<DockingConfig, TPriority> GetCandidateDockingConfigs<TPriority>(
        EntityUid stationUid,
        List<Entity<DockingComponent>> shuttleDocks,
        List<Entity<DockingComponent>> stationDocks,
        Box2 shuttleAABB,
        DockType dockType,
        IComparer<TPriority> comparer,
        Func<DockingConfig, TPriority> transformer)
    {
        var validDockConfigs = new Dictionary<
            (int, int, int, int, int, int),
            DockingConfig>();

        if (stationDocks.Count <= 0 || shuttleDocks.Count <= 0)
            return new();

        List<(float x, float y, Angle angle, Direction cardinal, EntityUid euid, DockingComponent dc, TransformComponent xform)> ToDockDetails(
            List<Entity<DockingComponent>> docks)
        {
            List<(float x, float y, Angle angle, Direction cardinal, EntityUid euid, DockingComponent dc, TransformComponent xform)> output = [];
            foreach (var (gridDockUid, gridDockComp) in docks)
            {
                if ((gridDockComp.DockType & dockType) == DockType.None)
                    continue;
                var dockXform = _xformQuery.GetComponent(gridDockUid);
                var dockLocalPos = dockXform.LocalPosition;

                output.Add((dockLocalPos.X, dockLocalPos.Y, dockXform.LocalRotation, dockXform.LocalRotation.GetCardinalDir(), gridDockUid, gridDockComp, dockXform));
            }
            return output;
        }

        var shuttleDockDetails = ToDockDetails(shuttleDocks);
        var stationDockDetails = ToDockDetails(stationDocks);

        foreach (var (stationDockX, stationDockY, stationDockAngle, stationDockCardinal, stationDockUid, stationDock, stationDockXform) in stationDockDetails)
        {
            var stationDockOppositeA = stationDockAngle.Opposite();
            foreach (var (shuttleDockX, shuttleDockY, shuttleDockAngle, shuttleDockCardinal, shuttleDockUid, shuttleDock, shuttleDockXform) in shuttleDockDetails)
            {
                if ((stationDock.DockType & shuttleDock.DockType) == DockType.None)
                    continue;

                // Triad: a berth may demand a specific shuttle side; see RequiredShuttleTag.
                if (!ShuttleSideAllowed(shuttleDockUid, stationDockUid))
                    continue;

                var matty = DockingConfig.CalcMatty(shuttleDockX,
                    shuttleDockY,
                    shuttleDockCardinal,
                    stationDockX,
                    stationDockY,
                    stationDockCardinal);

                var key = (
                    (int)(DockingConfig.MattyRadix * matty.M11),
                    (int)(DockingConfig.MattyRadix * matty.M12),
                    (int)(DockingConfig.MattyRadix * matty.M21),
                    (int)(DockingConfig.MattyRadix * matty.M22),
                    (int)(DockingConfig.MattyRadix * matty.M31),
                    (int)(DockingConfig.MattyRadix * matty.M32)
                    );

                if(validDockConfigs.TryGetValue(key, out var dockConfig))
                {
                    dockConfig.Docks.Add((shuttleDockUid, stationDockUid, shuttleDock, stationDock, shuttleDockXform, stationDockXform));
                }
                else
                {
                    validDockConfigs.Add(key,
                        new()
                    {
                        Docks = [(shuttleDockUid, stationDockUid, shuttleDock, stationDock, shuttleDockXform, stationDockXform)],
                        TargetGridUid = stationUid,
                        Angle = (stationDockOppositeA - shuttleDockAngle).Reduced(),
                        Area = matty.TransformBox(shuttleAABB),
                        Coordinates = new EntityCoordinates(stationUid, Vector2.Transform(Vector2.Zero, matty)),
                        Matty = matty
                    });
                }
            }
        }

        return new(validDockConfigs.Values.Select(t => (t, transformer(t))), comparer);
    }



}
