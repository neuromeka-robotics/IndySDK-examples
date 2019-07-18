#include "JointPDControl.h"

#define LOG_DATA_SAVE_PERIOD 10*4000

JointPDControl::JointPDControl()
: AbstractJointController<AbstractRobot6D>()
, _logCnt(LOG_DATA_SAVE_PERIOD)
, _time(0)
{
	_dataLogger.activate();
}

JointPDControl::~JointPDControl()
{
	_dataLogger.deactivate();
}

void JointPDControl::initialize()
{
}

void JointPDControl::setPeriod(double delt)
{
}

void JointPDControl::setPassivityMode(bool enable)
{
}

void JointPDControl::reset()
{


}

void JointPDControl::reset(int jIndex)
{

}

void JointPDControl::setGains(JointVec const & kp, JointVec const & kv, JointVec const & ki)
{

}

int JointPDControl::computeControlTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec const & qDesired, JointVec const & qdotDesired, JointVec const & qddotDesired, JointVec & torque)
{
	JointMat kp, kd;
	JointVec tauGrav;
	JointVec tauPD;

	// Initialize local variables
	kp.Zero();
	kd.Zero();
	tauPD.Zero();
	tauGrav.Zero();

	// Set PD control gains
	for (int i = 0; i < JOINT_DOF; ++i)
	{
		switch(i)
		{
		case 0:
		case 1:
			kp(i,i) = 70;
			kd(i,i) = 55;
			break;
		case 2:
			kp(i,i) = 40;
			kd(i,i) = 30;
			break;
		case 3:
		case 4:
			kp(i,i) = 25;
			kd(i,i) = 15;
			break;
		case 5:
			kp(i,i) = 18;
			kd(i,i) = 3;
			break;
		}
	}

	// Gravitational force calculation using inverse dynamics
	robot.idyn_gravity(LieGroup::Vector3D(0,0,-GRAV_ACC));
	tauGrav = robot.tau();

	// Joint-space PD control input
	tauPD = kp*(qDesired - robot.q()) - kd*robot.qdot() + tauGrav;

	// Update torque control input
	torque = tauPD;

	// Logging data in real-time
	_time += 0.0025;

	double q[JOINT_DOF], qdot[JOINT_DOF], tau[JOINT_DOF];

	for (int i = 0; i < JOINT_DOF; i++)
	{
		q[i] = robot.q()(i,0);
		qdot[i] = robot.qdot()(i,0);
		tau[i] = tauPD(i,0);
	}
	_dataLogger.updateLoggedData(_time, q, qdot, tau);

	// Triggering logger saving
	_logCnt--;
	if (_logCnt <= 0)
	{
		_dataLogger.triggerSaving();
		_logCnt = LOG_DATA_SAVE_PERIOD; // 10s
	}

	return 0;
}

int JointPDControl::computeGravityTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec & torque)
{
	// Compute gravitational torque
	JointVec tauGrav;
	robot.idyn_gravity(LieGroup::Vector3D(0, 0, -GRAV_ACC));
	tauGrav = robot.tau();

	// Update torque control input
	torque = tauGrav;

	return 0;
}


typedef JointPDControl JointGravityPDControl;

POCO_BEGIN_MANIFEST(AbstractJointController<AbstractRobot6D>)
	POCO_EXPORT_CLASS(JointGravityPDControl)
POCO_END_MANIFEST
