#ifndef MISSION_H
#define MISSION_H

class Mission
{
    public:
        enum class Type {
            GO = 0,
            TURN = 1,
            WAIT = 2,
            RESUME = 3,
            STOP = 4,
            NONE = 5
        };

        enum class Status {
            NOT_STARTED = 0,
            STARTED = 1,
            FINISHED = 2,
            CANCELED = 3
        };

        enum class Options {
            NONE = 0,
            SLOW = 1 << 0,
        };

        enum class Direction {
            FORWARD = 0,
            BACKWARD = 1
        };

        Mission(
            int id,
            Type type,
            Options options = Options::NONE,
            Direction direction = Direction::FORWARD,
            float target_x = 0.0f,
            float target_y = 0.0f,
            float target_theta = 0.0f
        ):
            id(id),
            type(type),
            options(options),
            direction(direction),
            target_x(target_x),
            target_y(target_y),
            target_theta(target_theta){}

        bool operator!=(const Mission& other) const { return this->id != other.id; }

        [[nodiscard]] int getId() const { return id; }

        [[nodiscard]] Type getType() const { return type; }

        [[nodiscard]] Status getStatus() const { return status; }

        void sendMisionUpdate()
        {
            String d = "M" + String(getId()) + ";" + String(static_cast<int>(getStatus()))+"F";
            Serial.println(d);
        }

        void setStatus(Status newStatus) {
            status = newStatus;
            sendMisionUpdate();
        }

        [[nodiscard]] Options getOptions() const { return options; }

        [[nodiscard]] float getTargetX() const { return target_x; }

        [[nodiscard]] float getTargetY() const { return target_y; }

        [[nodiscard]] float getTargetTheta() const { return target_theta; }

        [[nodiscard]] bool isActive() const { return status == Status::STARTED; }

        [[nodiscard]] bool isForward() const { return direction == Direction::FORWARD; }

        bool operator==(const Mission& other) const { return this->id == other.id; }

    static String typeToString(Mission::Type type) {
        switch (type) {
            case Mission::Type::GO:     return "GO";
            case Mission::Type::TURN:   return "TURN";
            case Mission::Type::WAIT:   return "WAIT";
            case Mission::Type::RESUME: return "RESUME";
            case Mission::Type::STOP:   return "STOP";
            case Mission::Type::NONE:   return "NONE";
            default:              return "UNKNOWN";
        }
    }

    void setThetaGoTo(float theta) { if(type == Mission::Type::GO) target_theta = theta; }

    protected:
        int id;
        Type type;
        Status status = Status::NOT_STARTED;
        Options options = Options::NONE;
        Direction direction = Direction::FORWARD;

        float target_x;
        float target_y;
        float target_theta;
};
#endif // MISSION_H