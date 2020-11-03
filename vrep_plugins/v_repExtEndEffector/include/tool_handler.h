#ifndef V_REPEXT_ENDEFFECTOR_TOOL_HANDLER
#define V_REPEXT_ENDEFFECTOR_TOOL_HANDLER

#include <map>
#include <string>
#include <vector>

enum TOOL_TYPES {
    NONE,
    EMPTY,
    CAMERA,
    PROBE,
    TREPAN
};

const std::map<TOOL_TYPES, std::string> tools_map {
    {NONE,   ""         },
    {EMPTY,  "eeempty"  },
    {CAMERA, "eecamera" },
    {PROBE,  "eeprobe"  },
    {TREPAN, "eetrepan" }
};


//Base Tool struct
struct Tool {
    int base_handle;
    std::string name;
    TOOL_TYPES type;
    bool has_joint;
    std::vector<float> origin_pos;
    std::vector<float> origin_ori;
    int joint_handle;
    std::pair<float, float> limits;
};

class ToolHandler {
public:
    //Populates Tool vector with types and required name
    ToolHandler();
    //Requires handles and other information from simulator about each tool
    bool InitTools();
    //Set/Replace tool
    void SetTool(const TOOL_TYPES tt);
    void RemoveTool();
    void MoveTool(const double dist);
    void GetToolPos(double &cur_pos);

private:
    int m_conn_handle;
    std::vector<Tool> m_tools;
    int m_current_tool; //index of current tool in m_tools
};

#endif //V_REPEXT_ENDEFFECTOR_TOOL_HANDLER

