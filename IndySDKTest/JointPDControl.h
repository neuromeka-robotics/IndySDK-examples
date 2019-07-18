#ifndef JointPDControl_H_
#define JointPDControl_H_

#include <NRMKFramework/Components/AbstractJointController.h>
#include <NRMKFramework/Components/AbstractRobot6D.h>

// Add data logger header
#include "DataLogger.h"

// Add shared memory
#include "NRMKFramework/shmem/ShmemManager.hpp"
#include "NRMKFramework/Indy/IndyTasksDefine.h"
#include "NRMKFramework/Indy/SharedMemory/SharedData.h"
#include "NRMKFramework/System/SharedMemory/SharedData.h"

class JointPDControl : public AbstractJointController<AbstractRobot6D>
{
	enum
	{
		JOINT_DOF = AbstractRobot6D::JOINT_DOF
	};
	typedef AbstractRobot6D ROBOT;
	typedef typename ROBOT::JointVec JointVec;
	typedef typename ROBOT::JointMat JointMat;

public:
	JointPDControl();
	~JointPDControl();

	void initialize();
	void setPeriod(double delt);
	void setPassivityMode(bool enable);
	void setGains(JointVec const & kp, JointVec const & kv, JointVec const & ki);
	void reset();
	void reset(int jIndex);

	int computeControlTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec const & qDesired, JointVec const & qdotDesired, JointVec const & qddotDesired, JointVec & torque);
	int computeGravityTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec & torque);

private:
	NRMKIndy::SharedData::RobotRTSharedData _rtData;
	NRMKIndy::SharedData::RobotControlStatusSharedData _ctrlStateData;
	NRMKIndy::SharedData::SmartDIOSharedData _smartDIO;
	DataLogger _dataLogger;
	unsigned int _logCnt;

private:
	double _time;

};

#endif /* JointPDControl_H_ */
