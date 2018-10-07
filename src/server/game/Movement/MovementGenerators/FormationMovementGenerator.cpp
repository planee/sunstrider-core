// rewritten for sunstrider
// - Main difference is that instead of going directly to dest, we also go to a start point with the correct offset. 
//   This ensures a trajectory parallel to the owner one
// 
// - TODO: This is still quite imprecise... I'd like to do a single path from start to dest point, but we can't handle variable speeds
//         I'd need some sniffs data to see how it's done so smoothly on retail

#include "FormationMovementGenerator.h"
#include "Creature.h"
#include "CreatureAI.h"
#include "MovementDefines.h"
#include "MoveSpline.h"
#include "MoveSplineInit.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

FormationMovementGenerator::FormationMovementGenerator(uint32 id, Creature* leader, FormationMoveSegment moveSegment) :
    MovementGeneratorMedium(MOTION_MODE_DEFAULT, MOTION_PRIORITY_NORMAL, UNIT_STATE_ROAMING),
    _movementId(id), 
    _leaderGUID(leader->GetGUID()),
    _moveSegment(moveSegment),
    _movingToStart(false),
    _skippedStart(false),
    _createTime(leader->GetMap()->GetGameTimeMS())
{
    Flags = MOVEMENTGENERATOR_FLAG_INITIALIZATION_PENDING;
}

MovementGeneratorType FormationMovementGenerator::GetMovementGeneratorType() const
{
    return FORMATION_MOTION_TYPE;
}

Position FormationMovementGenerator::GetMemberDestination(Creature* member, uint32 followDist, Position leaderDest, uint8 depth) const
{
    //no valid position found after 3 iterations, return leader position instead
    if (depth > 3)
        return leaderDest;

    float pathAngle = _moveSegment.start.GetAbsoluteAngle(_moveSegment.dest);

    Position dest;
    dest.m_orientation = leaderDest.GetOrientation();
    dest.m_positionX = leaderDest.GetPositionX() + std::cos(_moveSegment.followAngle + pathAngle) * followDist;
    dest.m_positionY = leaderDest.GetPositionY() + std::sin(_moveSegment.followAngle + pathAngle) * followDist;
    float height;
    if (!member->IsFlying())
        height = member->GetMap()->GetHeight(dest.m_positionX, dest.m_positionY, leaderDest.GetPositionZ() + 5.0f, true, 10.0f, member->GetCollisionHeight(), true);
    else
        height = leaderDest.GetPositionZ();

    if (height != INVALID_HEIGHT)
        dest.m_positionZ = height;
    else //no valid height found, try closer
        return GetMemberDestination(member, followDist / 2.0f, leaderDest, ++depth);

    Trinity::NormalizeMapCoord(dest.m_positionX);
    Trinity::NormalizeMapCoord(dest.m_positionY);

    return dest;
}

float FormationMovementGenerator::GetDistanceFromLine(Position point, Position start, Position end)
{
    typedef boost::geometry::model::d2::point_xy<double> point_type;
    typedef boost::geometry::model::linestring<point_type> linestring_type;

    point_type p(point.GetPositionX(), point.GetPositionY());
    linestring_type line;
    line.push_back(point_type(start.GetPositionX(), start.GetPositionY()));
    line.push_back(point_type(end.GetPositionX(), end.GetPositionY()));

    return boost::geometry::distance(p, line);
}

#define MAX_TIME_TO_START 2.0f

Creature* followedCreature = nullptr;

void FormationMovementGenerator::MoveToStart(Creature* owner)
{
    if (owner == followedCreature)
        owner = followedCreature;

    _movingToStart = true;
    
    // Decide if we should skip the start point, skip if we're:
    /*if (   start.GetExactDist(owner) < 2.5f                                    // - Close from the start point
        && owner->GetExactDist2d(_destPos) < start.GetExactDist2d(_destPos)      // - Closer to start than from end
        && GetDistanceFromLine(owner->GetPosition(), start, _destPos) < 3.0f)    // - Close to the line between start and end
    {
        _skippedStart = true;
        return; // This will trigger start to dest at next update
    }*/
           
    Movement::MoveSplineInit init(owner);

    bool walk = true;
    switch (_moveSegment.moveType)
    {
#ifdef LICH_KING
    case 2: // WAYPOINT_MOVE_TYPE_LAND
        init.SetAnimation(Movement::ToGround);
        break;
    case 3: // WAYPOINT_MOVE_TYPE_TAKEOFF
        init.SetAnimation(Movement::ToFly);
        break;
#endif
    case WAYPOINT_MOVE_TYPE_RUN:
        walk = false;
        break;
    case WAYPOINT_MOVE_TYPE_WALK:
        walk = true;
        break;
}
    init.SetWalk(walk);

    if (Creature* leader = owner->GetMap()->GetCreature(_leaderGUID))
    {
        // Increase velocity just a bit
        UnitMoveType mtype = walk ? MOVE_WALK : MOVE_RUN;
        float memberVelocity = leader->GetSpeed(mtype) * 1.2f;
        float distToStart = owner->GetExactDist(_startPos);
        float timeToStart = distToStart / memberVelocity; // in seconds
        if (timeToStart > MAX_TIME_TO_START)
        {   //Increase speed even more if we're far, we want to get there in MAX_TIME_TO_START
            memberVelocity = distToStart / MAX_TIME_TO_START;
            timeToStart = MAX_TIME_TO_START;
        }
        init.SetVelocity(memberVelocity);

        // Little adjust to destination:
        // We're trying to compensate for the fact that leader is already on his way when we're only going to our start position
        // Try to calculate how far the leader will be by the time we arrive and advance our destination with that distance
        // improve me: do it in 3D instead of 2D as this will lead to some imprecisions
        /*
        float const leaderVelocity = leader->GetSpeed(mtype);
        float const diffSinceLeaderStart = (leader->GetMap()->GetGameTimeMS() - _createTime) / 1000.0f; // in Seconds
        float const leaderSegmentLenght = _moveSegment.start.GetExactDist(_moveSegment.dest);
        //traveled distance by leader by the time we get to start. *2 because MoveToDest will also only be started one update after arrival and we want to compensate for that, altough this is very approximative and suppose constant diff on the map
        float leaderTraveledDistance = leaderVelocity * (timeToStart + diffSinceLeaderStart * (_skippedStart ? 1 : 2));
        leaderTraveledDistance = std::min(leaderSegmentLenght, leaderTraveledDistance);
        float const pathAngle = _moveSegment.start.GetAbsoluteAngle(_moveSegment.dest);
        float const newX = _startPos.GetPositionX() + std::cos(pathAngle) * leaderTraveledDistance;
        float const newY = _startPos.GetPositionY() + std::sin(pathAngle) * leaderTraveledDistance;
        _startPos.Relocate(newX, newY);
        owner->UpdateGroundPositionZ(_startPos.m_positionX, _startPos.m_positionY, _startPos.m_positionZ);*/
    }
    
    init.MoveTo(_startPos.GetPositionX(), _startPos.GetPositionY(), _startPos.GetPositionZ(), false, true);  //waypoint generator is not using mmaps... so neither should we
    if (_startPos.GetOrientation())
        init.SetFacing(_startPos.GetOrientation());

    init.Launch();
    owner->AddUnitState(UNIT_STATE_ROAMING_MOVE);
    RemoveFlag(MOVEMENTGENERATOR_FLAG_INTERRUPTED | MOVEMENTGENERATOR_FLAG_SPEED_UPDATE_PENDING);
}

void FormationMovementGenerator::MoveToDest(Creature* owner)
{
    if (owner == followedCreature)
        owner = followedCreature;

    _movingToStart = false;

    Movement::MoveSplineInit init(owner);
    init.MoveTo(_destPos.GetPositionX(), _destPos.GetPositionY(), _destPos.GetPositionZ(), false, true); //waypoint generator is not using mmaps... so neither should be
    if (_destPos.GetOrientation())
        init.SetFacing(_destPos.GetOrientation());

    bool walk = true;
    switch (_moveSegment.moveType)
    {
#ifdef LICH_KING
    case 2: // WAYPOINT_MOVE_TYPE_LAND
        init.SetAnimation(Movement::ToGround);
        break;
    case 3: // WAYPOINT_MOVE_TYPE_TAKEOFF
        init.SetAnimation(Movement::ToFly);
        break;
#endif
    case WAYPOINT_MOVE_TYPE_RUN:
        walk = false;
        break;
    case WAYPOINT_MOVE_TYPE_WALK:
        walk = true;
        break;
    }
    init.SetWalk(walk);

    if (Creature* leader = owner->GetMap()->GetCreature(_leaderGUID))
    {
        // change members speed basing on distance
        // -> What we aim here is arriving at the same time
        UnitMoveType mtype = walk ? MOVE_WALK : MOVE_RUN;
        float leaderSegmentLength = _moveSegment.start.GetExactDist(_moveSegment.dest);
        float memberSegmentLength = owner->GetExactDist(_destPos);
        float rate = std::max(1.0f, memberSegmentLength / leaderSegmentLength);
        init.SetVelocity(leader->GetSpeed(mtype) * rate);
    }

    init.Launch();
    owner->AddUnitState(UNIT_STATE_ROAMING_MOVE);
    RemoveFlag(MOVEMENTGENERATOR_FLAG_INTERRUPTED | MOVEMENTGENERATOR_FLAG_SPEED_UPDATE_PENDING);

    owner->SetHomePosition(POSITION_GET_X_Y_Z(&_destPos), 0.0f);
}

bool FormationMovementGenerator::DoInitialize(Creature* owner)
{
    if (owner == followedCreature)
        owner = followedCreature;

    _startPos = GetMemberDestination(owner, _moveSegment.followDist, _moveSegment.start);
    _destPos = GetMemberDestination(owner, _moveSegment.followDist, _moveSegment.dest);

    _previousHome = owner->GetHomePosition();
    RemoveFlag(MOVEMENTGENERATOR_FLAG_INITIALIZATION_PENDING | MOVEMENTGENERATOR_FLAG_TRANSITORY | MOVEMENTGENERATOR_FLAG_DEACTIVATED);
    AddFlag(MOVEMENTGENERATOR_FLAG_INITIALIZED);

    if (owner->HasUnitState(UNIT_STATE_NOT_MOVE) || owner->IsMovementPreventedByCasting())
    {
        AddFlag(MOVEMENTGENERATOR_FLAG_INTERRUPTED);
        owner->StopMoving();
        return true;
    }

    MoveToStart(owner);
    return true;
}

void FormationMovementGenerator::DoReset(Creature* owner)
{
    RemoveFlag(MOVEMENTGENERATOR_FLAG_TRANSITORY | MOVEMENTGENERATOR_FLAG_DEACTIVATED);

    DoInitialize(owner);
}

bool FormationMovementGenerator::DoUpdate(Creature* owner, uint32 /*diff*/)
{
    if (!owner)
        return false;

    if (owner == followedCreature)
        owner = followedCreature;

    if (owner->HasUnitState(UNIT_STATE_NOT_MOVE) || owner->IsMovementPreventedByCasting())
    {
        AddFlag(MOVEMENTGENERATOR_FLAG_INTERRUPTED);
        owner->StopMoving();
        return true;
    }

    if (   (_movingToStart && owner->movespline->Finalized())
        || (HasFlag(MOVEMENTGENERATOR_FLAG_INTERRUPTED) && owner->movespline->Finalized())
        || (HasFlag(MOVEMENTGENERATOR_FLAG_SPEED_UPDATE_PENDING) && !owner->movespline->Finalized()))
    {
        MoveToDest(owner);
    }

    if (owner->movespline->Finalized())
    {
        RemoveFlag(MOVEMENTGENERATOR_FLAG_TRANSITORY);
        AddFlag(MOVEMENTGENERATOR_FLAG_INFORM_ENABLED);
        return false;
    }
    return true;
}

void FormationMovementGenerator::DoDeactivate(Creature* owner)
{
    owner->SetHomePosition(_previousHome);
    AddFlag(MOVEMENTGENERATOR_FLAG_DEACTIVATED);
    owner->ClearUnitState(UNIT_STATE_ROAMING_MOVE);
}

void FormationMovementGenerator::DoFinalize(Creature* owner, bool active, bool movementInform)
{
    AddFlag(MOVEMENTGENERATOR_FLAG_FINALIZED);
    if (active)
        owner->ClearUnitState(UNIT_STATE_ROAMING_MOVE);

    if (movementInform && HasFlag(MOVEMENTGENERATOR_FLAG_INFORM_ENABLED))
        MovementInform(owner);
}

void FormationMovementGenerator::MovementInform(Creature* owner)
{
    if (owner->AI())
        owner->AI()->MovementInform(FORMATION_MOTION_TYPE, _movementId);
}
