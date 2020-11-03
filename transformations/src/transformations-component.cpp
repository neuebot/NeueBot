#include "transformations-component.hpp"
#include <rtt/Component.hpp>
#include <rtt/marsh/Marshalling.hpp>

#include <cstring>
#include <cstdlib>
#include <iostream>

#define TODEG(val) (180*val/M_PI)
#define TORAD(val) (M_PI*val/180)

using namespace KDL;
using namespace RTT;
using namespace std;

Transformations::Transformations(const string &component_name) :
    TaskContext(component_name, PreOperational)
{
    //End effectors
    this->addProperty("None", m_ee_none_v).doc("End effector 'none'.");
    this->addProperty("Empty", m_ee_empty_v).doc("End effector 'empty'.");
    this->addProperty("Probe", m_ee_probe_v).doc("End effector 'probe'.");
    this->addProperty("Trepan", m_ee_trepan_v).doc("End effector 'trepan'.");
    this->addProperty("EEBase", m_ee_base_v).doc("End effector 'interlock base'.");
    this->addProperty("BaseButtons", m_ee_basebuttons_v).doc("End effector 'interlock base buttons'.");
    this->addProperty("BaseProbe", m_ee_baseprobe_v).doc("End effector 'interlock base probe'.");
    this->addProperty("BaseTrepan", m_ee_basetrepan_v).doc("End effector 'interlock base trepan'.");
    this->addProperty("Pointer", m_ee_pointer_v).doc("End effector 'pointer'.");

    this->addOperation("SetWorkFrame", &Transformations::SetWorkFrame, this, OwnThread)
        .doc("Sets the Base to Work (surgery) transformation used by the controller to define robot motions")
        .arg("base_work","KDL.Frame with transformation from base frame to surgery frame");
    this->addOperation("SetToolType", &Transformations::SetToolType, this, OwnThread);
    this->addOperation("GetToolType", &Transformations::GetToolType, this, ClientThread);
    this->addOperation("GetToolFrame", &Transformations::GetToolFrame, this, ClientThread);
    this->addOperation("GetToolTypeFrame", &Transformations::GetToolTypeFrame, this, ClientThread);
    this->addOperation("GetReferenceFrame", &Transformations::GetReferenceFrame, this, ClientThread);

    this->ports()->addEventPort("inport_robot_posture", inport_robot_posture);
    this->ports()->addPort("outport_error_msg", outport_error_msg);
}

bool Transformations::configureHook() {
    //Properties Marshalling
    if(const char* WS = std::getenv("WORKSPACE")) {
        char *TP = new char[128];
        strcpy(TP, WS);
        strcat(TP, "/Properties/TOOLS_PROPERTIES.xml");
        if(!this->getProvider<Marshalling>("marshalling")->loadProperties(TP)) {
            log(Error) << "Properties not loaded" << endlog();
            return false;
        }

        delete[] TP;
    }

    //Defining end-effector matrices
    m_ee_none = Frame(Rotation::Quaternion(m_ee_none_v[3], m_ee_none_v[4], m_ee_none_v[5], m_ee_none_v[6]),
            Vector(m_ee_none_v[0], m_ee_none_v[1], m_ee_none_v[2]));
    m_ee_empty = Frame(Rotation::Quaternion(m_ee_empty_v[3], m_ee_empty_v[4], m_ee_empty_v[5], m_ee_empty_v[6]),
            Vector(m_ee_empty_v[0], m_ee_empty_v[1], m_ee_empty_v[2]));
    m_ee_probe = Frame(Rotation::Quaternion(m_ee_probe_v[3], m_ee_probe_v[4], m_ee_probe_v[5], m_ee_probe_v[6]),
            Vector(m_ee_probe_v[0], m_ee_probe_v[1], m_ee_probe_v[2]));
    m_ee_trepan = Frame(Rotation::Quaternion(m_ee_trepan_v[3], m_ee_trepan_v[4], m_ee_trepan_v[5], m_ee_trepan_v[6]),
            Vector(m_ee_trepan_v[0], m_ee_trepan_v[1], m_ee_trepan_v[2]));
    m_ee_base = Frame(Rotation::Quaternion(m_ee_base_v[3], m_ee_base_v[4], m_ee_base_v[5], m_ee_base_v[6]),
            Vector(m_ee_base_v[0], m_ee_base_v[1], m_ee_base_v[2]));
    m_ee_basebuttons = Frame(Rotation::Quaternion(m_ee_basebuttons_v[3], m_ee_basebuttons_v[4], m_ee_basebuttons_v[5], m_ee_basebuttons_v[6]),
            Vector(m_ee_basebuttons_v[0], m_ee_basebuttons_v[1], m_ee_basebuttons_v[2]));
    m_ee_baseprobe = Frame(Rotation::Quaternion(m_ee_baseprobe_v[3], m_ee_baseprobe_v[4], m_ee_baseprobe_v[5], m_ee_baseprobe_v[6]),
            Vector(m_ee_baseprobe_v[0], m_ee_baseprobe_v[1], m_ee_baseprobe_v[2]));
    m_ee_basetrepan = Frame(Rotation::Quaternion(m_ee_basetrepan_v[3], m_ee_basetrepan_v[4], m_ee_basetrepan_v[5], m_ee_basetrepan_v[6]),
            Vector(m_ee_basetrepan_v[0], m_ee_basetrepan_v[1], m_ee_basetrepan_v[2]));
    m_ee_pointer = Frame(Rotation::Quaternion(m_ee_pointer_v[3], m_ee_pointer_v[4], m_ee_pointer_v[5], m_ee_pointer_v[6]),
            Vector(m_ee_pointer_v[0], m_ee_pointer_v[1], m_ee_pointer_v[2]));

    if(hasPeer("robot_state")) {
        TaskContext *rs_ptr = getPeer("robot_state");
        CallSetFlangeToolFrame = rs_ptr->getOperation("SetFlangeToolFrame");
        CallSetWorkBaseFrame = rs_ptr->getOperation("SetWorkBaseFrame");
        //Set engine caller of send operations (run in component)
        CallSetFlangeToolFrame.setCaller(rs_ptr->engine());
        CallSetWorkBaseFrame.setCaller(rs_ptr->engine());
    }
    else {
        log(Error) << "Could not connect transformations to robot_state component." << endlog();
        return false;
    }

    if(hasPeer("test_supervisor")) {
        TaskContext *ts_ptr = getPeer("test_supervisor");
        a_simulation = ts_ptr->getAttribute("m_simulation");
    }
    else {
        log(Error) << "Could not connect transformations to test_supervisor component." << endlog();
        return false;
    }

    if(hasPeer("test_vrep_fri")) {
        TaskContext *tvf_ptr = getPeer("test_vrep_fri");
        CallGetCurrentEndEffector = tvf_ptr->getOperation("GetCurrentEndEffector");
        CallAttachEndEffector = tvf_ptr->getOperation("AttachEndEffector");
        //To send and collect data (run in transformations enging 'OwnThread')
        CallGetCurrentEndEffector.setCaller(tvf_ptr->engine());
        CallAttachEndEffector.setCaller(tvf_ptr->engine());
    }

    // Port samples
    error_msg e;
    e.level = 9999;
    e.msg = string(256, '\0');
    outport_error_msg.setDataSample(e);

    return true;
}

bool Transformations::startHook(){
    m_work_transf = Frame::Identity(); //Not defined
    m_flange_transf = Frame::Identity(); //Not defined
    m_tool_transf = Frame(m_ee_none);

    m_end_effector_id = NONE;

    return true;
}

void Transformations::updateHook(){
    posture posture;
    if(inport_robot_posture.read(posture) == NewData) {
        m_flange_transf = posture.frame;
    }
}

void Transformations::stopHook() {
}

void Transformations::cleanupHook() {
}

void Transformations::errorHook() {
    log(Warning) << "BEEN HERE! Transformations ErrorHook()" << endlog();
    recover();
}

void Transformations::SetWorkFrame(const Frame base_work) {
    m_work_transf = base_work;

    // Send Work Base Transformation (inverse of m_work_transf) to Robot State
    KDL::Frame work_base = base_work.Inverse();
    CallSetWorkBaseFrame.call(work_base);

    error_msg emsg;
    emsg.level = WRKREFSET;
    emsg.msg = m_ec.dict[WRKREFSET];
    outport_error_msg.write(emsg);
}

bool Transformations::SetToolType(const int type) {
    error_msg emsg;
    switch(type) {
    case END_EFFECTORS::NONE:
        m_end_effector_id = END_EFFECTORS::NONE;
        m_tool_transf = m_ee_none;
        emsg.level = NONEEESET;
        emsg.msg = m_ec.dict[NONEEESET];
        break;
    case END_EFFECTORS::EMPTY:
        m_end_effector_id = END_EFFECTORS::EMPTY;
        m_tool_transf = m_ee_empty;
        emsg.level = EMPTEESET;
        emsg.msg = m_ec.dict[EMPTEESET];
        break;
    case END_EFFECTORS::CAMERA:
        m_end_effector_id = END_EFFECTORS::CAMERA;
        m_tool_transf = m_ee_empty; //TODO
        emsg.level = CAMEEESET;
        emsg.msg = m_ec.dict[CAMEEESET];
        break;
    case END_EFFECTORS::PROBE:
        m_end_effector_id = END_EFFECTORS::PROBE;
        m_tool_transf = m_ee_probe;
        emsg.level = PROBEESET;
        emsg.msg = m_ec.dict[PROBEESET];
        break;
    case END_EFFECTORS::TREPAN:
        m_end_effector_id = END_EFFECTORS::TREPAN;
        m_tool_transf = m_ee_trepan;
        emsg.level = TREPEESET;
        emsg.msg = m_ec.dict[TREPEESET];
        break;
    case END_EFFECTORS::EEBASE:
        m_end_effector_id = END_EFFECTORS::EEBASE;
        m_tool_transf = m_ee_base;
        emsg.level = EEBASESET;
        emsg.msg = m_ec.dict[EEBASESET];
        break;
    case END_EFFECTORS::BASEBUTTONS:
        m_end_effector_id = END_EFFECTORS::BASEBUTTONS;
        m_tool_transf = m_ee_basebuttons;
        emsg.level = BBUTTNSET;
        emsg.msg = m_ec.dict[BBUTTNSET];
        break;
    case END_EFFECTORS::BASEPROBE:
        m_end_effector_id = END_EFFECTORS::BASEPROBE;
        m_tool_transf = m_ee_baseprobe;
        emsg.level = BPROBESET;
        emsg.msg = m_ec.dict[BPROBESET];
        break;
    case END_EFFECTORS::BASETREPAN:
        m_end_effector_id = END_EFFECTORS::BASETREPAN;
        m_tool_transf = m_ee_basetrepan;
        emsg.level = BTREPNSET;
        emsg.msg = m_ec.dict[BTREPNSET];
        break;
    case END_EFFECTORS::POINTER:
        m_end_effector_id = END_EFFECTORS::POINTER;
        m_tool_transf = m_ee_pointer;
        emsg.level = PNTREESET;
        emsg.msg = m_ec.dict[PNTREESET];
        break;
    default:
        emsg.level = INEXISTEE;
        emsg.msg = m_ec.dict[INEXISTEE];
        outport_error_msg.write(emsg);
        return false;
    }

//    if(a_simulation.get()) {
        // Attach to VREP simulation
        CallAttachEndEffector.call(type);
//    }
    // Send new tool transformation to Robot State component
    CallSetFlangeToolFrame.call(m_tool_transf);

    outport_error_msg.write(emsg);
    return true;
}

int Transformations::GetToolType() {
    //TODO: Change to simulation
    if(a_simulation.get()) {
        int tool_type = CallGetCurrentEndEffector.call();

        m_end_effector_id = static_cast<END_EFFECTORS>(tool_type);
    }

    return m_end_effector_id;
}

void Transformations::GetToolFrame(Frame &flange_tool) {
    flange_tool = m_tool_transf;
}

void Transformations::GetToolTypeFrame(const int type, Frame &flange_tool) {
    error_msg err;
    switch(type) {
    case END_EFFECTORS::NONE:
        flange_tool = KDL::Frame::Identity();
        break;
    case END_EFFECTORS::EMPTY:
        flange_tool = m_ee_empty;
        break;
    case END_EFFECTORS::CAMERA:
        flange_tool = m_ee_empty;
        break;
    case END_EFFECTORS::PROBE:
        flange_tool = m_ee_probe;
        break;
    case END_EFFECTORS::TREPAN:
        flange_tool = m_ee_trepan;
        break;

    case END_EFFECTORS::EEBASE:
        flange_tool = m_ee_base;
        break;
    case END_EFFECTORS::BASEBUTTONS:
        flange_tool = m_ee_basebuttons;
        break;
    case END_EFFECTORS::BASEPROBE:
        flange_tool = m_ee_baseprobe;
        break;

    case END_EFFECTORS::BASETREPAN:
        flange_tool = m_ee_basetrepan;
        break;

    case END_EFFECTORS::POINTER:
        flange_tool = m_ee_pointer;
        break;
    default:
        flange_tool = KDL::Frame::Identity();
        err.level = UNIDENTFR;
        err.msg = m_ec.dict[UNIDENTFR];
        outport_error_msg.write(err);
    }
}

bool Transformations::GetReferenceFrame(const int reference_id, Frame &frame) {
    error_msg emsg;
    switch(reference_id) {
    case REFERENCE_FRAME::BASE:
        frame = Frame::Identity();
        break;
    case REFERENCE_FRAME::FLANGE:
        frame = m_flange_transf;

//        log(Warning) << "Pos " << frame.p.x() << " " << frame.p.y() << " " << frame.p.z() << endl;

        break;
    case REFERENCE_FRAME::END_EFFECTOR:
        frame = m_flange_transf * m_tool_transf;
        break;
    case REFERENCE_FRAME::WORK:
        if(m_work_transf == KDL::Frame::Identity()) {
            emsg.level = NOCALIBRS;
            emsg.msg = m_ec.dict[NOCALIBRS];
            outport_error_msg.write(emsg);
        }
        frame = m_work_transf;
        break;
    default:
        emsg.level = INEXISTRF;
        emsg.msg = m_ec.dict[INEXISTRF];
        outport_error_msg.write(emsg);
        return false;
    }
    return true;
}

ORO_CREATE_COMPONENT(Transformations)
