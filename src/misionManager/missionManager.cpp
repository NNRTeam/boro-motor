#include "missionManager/missionManager.h"

missionManager::missionManager(Logger& logger): logger(logger){}

void missionManager::addMission(Mission const &mission)
{
    logger.info("Adding mission ID: " + String(mission.getId()) +
                " Type: " + Mission::typeToString(mission.getType()));
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
        logger.info("Ending mission ID: " + String(currentMission->getId()));
        missions.erase(currentMission);
    }
}

[[nodiscard]] Mission* missionManager::startNextMission()
{
    if (missions.empty())
        return nullptr;

    auto *currentMission = getCurrentMission();
    currentMission->setStatus(Mission::Status::STARTED);
    logger.info("Starting mission ID: " + String(currentMission->getId()));
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
    return !missions.empty() && getCurrentMission()->isActive();
}

[[nodiscard]] Mission::Type missionManager::getCurrentMissionType() const
{
    if (missions.empty())
        return Mission::Type::NONE;

    return missions.front().getType();
}