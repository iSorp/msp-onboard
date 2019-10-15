
#pragma once


class Mavlink;

class MavlinkMissionManager
{
public:
	explicit MavlinkMissionManager(Mavlink *mavlink);

	~MavlinkMissionManager() = default;

	void send();
	void handle_message(const mavlink_message_t *msg);
	void check_active_mission(void);

private:
    Mavlink *_mavlink;
	
};
