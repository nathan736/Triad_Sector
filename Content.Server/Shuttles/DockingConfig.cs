using System.Numerics;
using System.Text;
using Content.Server.Shuttles.Components;
using Robust.Shared.Map;

namespace Content.Server.Shuttles;

/// <summary>
/// Stores the data for a valid docking configuration for the emergency shuttle
/// </summary>
public sealed class DockingConfig
{

    public const int MattyRadix = 1 << 4;
    /// <summary>
    /// The pairs of docks that can connect.
    ///
    /// triad: formerly DockAUid, DockBUid, DockA, DockB
    /// Also, didn't include the transform component
    /// </summary>
    public List<(
        EntityUid ShuttleDockUid,
        EntityUid StationDockUid,
        DockingComponent ShuttleDock,
        DockingComponent StationDock,
        TransformComponent ShuttleDockXform,
        TransformComponent StationDockXform)> Docks = new();

    /// <summary>
    /// Target grid for docking.
    /// It contains the AABB of the docking grid in the coordinate space of the target grid
    /// </summary>
    public EntityUid TargetGridUid;

    /// <summary>
    /// This is used for debugging.
    /// </summary>
    public Box2 Area;

    public EntityCoordinates Coordinates;

    /// <summary>
    /// Local angle of the docking grid relative to the target grid.
    /// </summary>
    public Angle Angle;


    // The transform needed to place the shuttle into the station's local space next to the dock.
    public Matrix3x2 Matty;

    public void SetMatty(
        TransformComponent shuttleXform,
        TransformComponent stationXform)
    {
        Matty = CalcMatty(
            shuttleXform,
            stationXform
        );
    }

    public static Matrix3x2 CalcMatty(
        TransformComponent shuttleXform,
        TransformComponent stationXform
    )
    {
        var shuttleLocalPos = shuttleXform.LocalPosition;
        var stationLocalPos = stationXform.LocalPosition;
        return CalcMatty(
            shuttleLocalPos.X,
            shuttleLocalPos.Y,
            shuttleXform.LocalRotation.GetCardinalDir(),
            stationLocalPos.X,
            stationLocalPos.Y,
            stationXform.LocalRotation.GetCardinalDir()
        );
    }

    public void SetMatty(
        float shuttleDockX,
        float shuttleDockY,
        Direction shuttleDockDir,
        float stationDockX,
        float stationDockY,
        Direction stationDockDir)
    {
        Matty = CalcMatty(
            shuttleDockX,
            shuttleDockY,
            shuttleDockDir,
            stationDockX,
            stationDockY,
            stationDockDir
        );
    }
    public static Matrix3x2 CalcMatty(
        float shuttleDockX,
        float shuttleDockY,
        Direction shuttleDockDir,
        float stationDockX,
        float stationDockY,
        Direction stationDockDir)
    {
        SinCosI(shuttleDockDir, out var shuttleDockSin, out var shuttleDockCos);
        SinCosI(stationDockDir, out var stationDockSin, out var stationDockCos);
        int stationDockOppositeSin = -stationDockSin;
        int stationDockOppositeCos = -stationDockCos;

        shuttleDockX = MathF.Round(shuttleDockX*MattyRadix, MidpointRounding.AwayFromZero)/MattyRadix;
        shuttleDockY = MathF.Round(shuttleDockY*MattyRadix, MidpointRounding.AwayFromZero)/MattyRadix;
        stationDockX = MathF.Round(stationDockX*MattyRadix, MidpointRounding.AwayFromZero)/MattyRadix;
        stationDockY = MathF.Round(stationDockY*MattyRadix, MidpointRounding.AwayFromZero)/MattyRadix;

        var m11 = shuttleDockCos * stationDockOppositeCos + shuttleDockSin * stationDockOppositeSin; // [-1, 0, 1]
        var m12 = shuttleDockCos * stationDockOppositeSin - shuttleDockSin * stationDockOppositeCos; // [-1, 0, 1]

        var m21 = shuttleDockSin * stationDockOppositeCos - shuttleDockCos * stationDockOppositeSin; // [-1, 0, 1]
        var m22 = shuttleDockSin * stationDockOppositeSin + shuttleDockCos * stationDockOppositeCos; // [-1, 0, 1]

        var m31 =
            - shuttleDockX * m11
            - shuttleDockY * m21
            + stationDockX
            - stationDockOppositeSin // offset so the shuttle dock lands next to the station dock
            ;

        var m32 =
            - shuttleDockX * m12
            - shuttleDockY * m22
            + stationDockY
            + stationDockOppositeCos // offset so the shuttle dock lands next to the station dock
            ;

        return new Matrix3x2()
        {
            M11 = m11,
            M12 = m12,
            M21 = m21,
            M22 = m22,
            M31 = m31,
            M32 = m32,
        };

        // We are assuming that all docks are anchored and are cardinal.
        void SinCosI(Direction dir, out int sin, out int cos)
        {
            switch (dir)
            {
                case Direction.SouthWest:
                case Direction.South:
                    sin = 0;
                    cos = 1;
                    break;
                case Direction.SouthEast:
                case Direction.East:
                    sin = 1;
                    cos = 0;
                    break;
                case Direction.NorthEast:
                case Direction.North:
                    sin = 0;
                    cos = -1;
                    break;
                case Direction.NorthWest:
                case Direction.West:
                    sin = -1;
                    cos = 0;
                    break;
                default:
                    sin = 0;
                    cos = 0;
                    break;
            }
        }
    }
}
