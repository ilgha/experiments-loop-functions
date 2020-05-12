#include "MarkerAggregationLoopFunc.h"

using namespace argos;

constexpr float MarkerAggregationLoopFunction::_marker_radius;
constexpr float MarkerAggregationLoopFunction::_valid_area_radius;

MarkerAggregationLoopFunction::MarkerAggregationLoopFunction() :
    _objective_function(0.0f)
{}

MarkerAggregationLoopFunction::~MarkerAggregationLoopFunction()
{}

void MarkerAggregationLoopFunction::Destroy()
{}

void MarkerAggregationLoopFunction::Init(TConfigurationNode& t_tree)
{
    CoreLoopFunctions::Init(t_tree);
}

argos::CColor MarkerAggregationLoopFunction::GetFloorColor(
    const argos::CVector2& c_position_on_plane)
{
    return (c_position_on_plane.Length() <= _marker_radius) ?
        CColor::BLACK :
        CColor::GRAY50;
}

void MarkerAggregationLoopFunction::PostStep()
{
    auto& epuck_map = GetSpace().GetEntitiesByType("epuck");
    int epuck_count_in_valid_pos = 0;

    // count the number of robots in valid area
    for(auto it = epuck_map.begin(); it != epuck_map.end(); ++it)
    {
        auto epuck = any_cast<CEPuckEntity*>(it->second);
        auto epuck_pos3 = epuck->GetEmbodiedEntity().GetOriginAnchor().Position;
        CVector2 epuck_pos(epuck_pos3.GetX(), epuck_pos3.GetY());

        // test if robot is in area
        if(epuck_pos.Length() < _valid_area_radius)
        {
            ++epuck_count_in_valid_pos;
        }
    }

    // compute step score
    Real step_score = epuck_count_in_valid_pos;
    LOG << "step score: " << step_score << std::endl;

    // update global score
    _objective_function += step_score;
}

void MarkerAggregationLoopFunction::Reset()
{
    CoreLoopFunctions::Reset();
    _objective_function = 0.0f;
}

CVector3 MarkerAggregationLoopFunction::GetRandomPosition()
{
    Real temp;
    Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    Real b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    // If b < a, swap them
    if (b < a) {
    temp = a;
    a = b;
    b = temp;
    }
    Real fPosX = b * m_fDistributionRadius *
        cos(2.0f * CRadians::PI.GetValue() * (a/b));
    Real fPosY = b * m_fDistributionRadius *
        sin(2.0f * CRadians::PI.GetValue() * (a/b));

    return CVector3(fPosX, fPosY, 0);
}

REGISTER_LOOP_FUNCTIONS(MarkerAggregationLoopFunction,
                        "marker_aggregation_loop_function");

