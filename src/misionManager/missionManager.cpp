#include "missionManager/missionManager.h"
#include <Utils.h>
#include <algorithm>

missionManager::missionManager(Logger& logger): m_logger(logger){
}

void missionManager::addMission(Mission const &mission)
{
    m_logger.info("Adding mission ID: " + String(mission.getId()) +
                " Type: " + Mission::typeToString(mission.getType()));
    if (mission.getType() == Mission::Type::STOP)
    {
        m_logger.info("STOP mission received. Clearing all missions.");
        clearMissions();
        return;
    }
    else if (mission.getType() == Mission::Type::RESUME)
    {
        if (getCurrentMission()->getType() == Mission::Type::WAIT)
        {
            endCurrentMission();
            startNextMission();
        }
        else
        {
            m_logger.warn("RESUME mission received but current mission is not WAIT.");
        }
        return;
    }
    missions.push_back(mission);
}

void missionManager::clearMissions()
{
    missions.clear();
}

[[nodiscard]] bool missionManager::removeMission(Mission &mission)
{
    auto it = std::find(missions.begin(), missions.end(), mission);
    if (it != missions.end())
    {
        for (auto &mission: missions)
        {
            mission.setStatus(Mission::Status::CANCELED);
        }
        missions.clear();
        return true;
    }
    return false;
}

[[nodiscard]] Mission* missionManager::getCurrentMission()
{
    if (missions.empty())
        return nullptr;
    return &missions.front();
}

void missionManager::endCurrentMission()
{
    if (hasActiveMissions())
    {
        auto *currentMission = getCurrentMission();
        currentMission->setStatus(Mission::Status::FINISHED);
        m_logger.info("Ending mission ID: " + String(currentMission->getId()));
        auto it = std::find(missions.begin(), missions.end(), *currentMission);
        if (it != missions.end()) {
            missions.erase(it);
        }
    }
}

Mission* missionManager::startNextMission()
{
    if (missions.empty())
        return nullptr;

    auto *currentMission = getCurrentMission();
    currentMission->setStatus(Mission::Status::STARTED);
    m_logger.info("Starting mission ID: " + String(currentMission->getId()));
    return currentMission;
}

[[nodiscard]] Mission* missionManager::getMissionById(int id)
{
    for (auto &mission : missions)
    {
        if (mission.getId() == id)
        {
            return &mission;
        }
    }
    return nullptr;
}

[[nodiscard]] size_t missionManager::getMissionCount() const
{
    return missions.size();
}

[[nodiscard]] bool missionManager::hasMissions() const
{
    return !missions.empty();
}

[[nodiscard]] bool missionManager::hasActiveMissions()
{
    bool const hasActiveMissions = !missions.empty() && getCurrentMission()->isActive();
    return hasActiveMissions;
}

[[nodiscard]] Mission::Type missionManager::getCurrentMissionType() const
{
    if (missions.empty())
        return Mission::Type::NONE;

    return missions.front().getType();
}

[[nodiscard]] Mission* missionManager::parseMissionMessage(String const &message)
{
    // message format: "{ID};{TYPE_INT};{STATUS_INT};{OPTION_INT};{DIRECTION_INT};{TARGET_X_FLOAT};{TARGET_Y_FLOAT};{TARGET_THETA_FLOAT}"
    auto parts = std::vector<String>();
    int start = 0;
    for (size_t i = 0; i < message.length(); ++i)
    {
        if (message.charAt(i) == ';')
        {
            parts.push_back(message.substring(start, i));
            start = i + 1;
        }
    }
    parts.push_back(message.substring(start));
    if (parts.size() != 8)
    {
        m_logger.error("Invalid mission message format: " + message);
        return nullptr;
    }
    int id = parts[0].toInt();
    Mission::Type type = static_cast<Mission::Type>(parts[1].toInt());
    Mission::Status status = static_cast<Mission::Status>(parts[2].toInt());
    Mission::Options options = static_cast<Mission::Options>(parts[3].toInt());
    Mission::Direction direction = static_cast<Mission::Direction>(parts[4].toInt());
    float target_x = parts[5].toFloat();
    float target_y = parts[6].toFloat();
    float target_theta = parts[7].toFloat();
    Mission* mission = new Mission(id, type, options, direction, target_x, target_y, target_theta);
    mission->setStatus(status);
    return mission;
}

void missionManager::cancelAllMissions()
{
    for (auto &mission: missions)
        mission.setStatus(Mission::Status::CANCELED);

    missions.clear();
}

void missionManager::addFakeMissionForTest()
{
    Mission mission1(1, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.5f, 0.0f, 0.0f);
    addMission(mission1);
    Mission mission2(2, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 3.14159f/2.0f);
    addMission(mission2);
    Mission mission3(3, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.5f, 0.5f, 0.0f);
    addMission(mission3);
    Mission mission4(4, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 3.14159f);
    addMission(mission4);
    Mission mission5(5, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.5f, 0.0f);
    addMission(mission5);
    Mission mission6(6, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, -3.14159f/2.0f);
    addMission(mission6);
    Mission mission7(7, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 0.0f);
    addMission(mission7);
    Mission mission8(8, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 0.0f);
    addMission(mission8);

    Mission mission11(11, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.5f, 0.0f, 0.0f);
    addMission(mission11);
    Mission mission12(12, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 3.14159f/2.0f);
    addMission(mission12);
    Mission mission13(13, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.5f, 0.5f, 0.0f);
    addMission(mission13);
    Mission mission14(14, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 3.14159f);
    addMission(mission14);
    Mission mission15(15, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.5f, 0.0f);
    addMission(mission15);
    Mission mission16(16, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, -3.14159f/2.0f);
    addMission(mission16);
    Mission mission17(17, Mission::Type::GO, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 0.0f);
    addMission(mission17);
    Mission mission18(18, Mission::Type::TURN, Mission::Options::NONE, Mission::Direction::FORWARD, 0.0f, 0.0f, 0.0f);
    addMission(mission18);
}
