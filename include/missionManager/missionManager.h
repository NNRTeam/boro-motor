#ifndef MISSIONMANAGER_H
#define MISSIONMANAGER_H

#include <Arduino.h>
#include <vector>
#include <missionManager/mission.h>
#include <Logger/Logger.h>

class missionManager
{
public:
    missionManager(Logger& logger);

    void addMission(Mission const &mission);
    void clearMissions();
    [[nodiscard]] bool removeMission(Mission &mission);
    [[nodiscard]] Mission* getCurrentMission();
    void endCurrentMission();
    [[nodiscard]] Mission* startNextMission();
    [[nodiscard]] Mission* getMissionById(int id);
    [[nodiscard]] size_t getMissionCount() const;
    [[nodiscard]] bool hasMissions() const;
    [[nodiscard]] bool hasActiveMissions();
    [[nodiscard]] Mission::Type getCurrentMissionType() const;
protected:
    std::vector<Mission> missions;

private:
    Logger& logger;
};





#endif // MISSIONMANAGER_H