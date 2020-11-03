#include "kdl_marshaller.h"

using namespace boost::property_tree;

using namespace std;

KDL::Marshaller::Marshaller()
{
}

void KDL::Marshaller::LoadFile(const std::string &file_name)
{
    m_root.clear();

    try {
        read_json(file_name, m_root);
    } catch (std::exception &e) {
        cout << e.what() << endl;
    }
}

void KDL::Marshaller::SaveFile(const string &file_name)
{
    try {
        write_json(file_name, m_root);
    } catch (std::exception &e) {
        cout << e.what() << endl;
    }
}

std::vector<double> KDL::Marshaller::get_std_vector(ptree &std_vector_json)
{
    std::vector<double> values;
    // Iterator over all position coordinates
    for (ptree::value_type &value : std_vector_json)
    {
        values.push_back(value.second.get_value<double>());
    }

    return values;
}

KDL::Vector KDL::Marshaller::get_kdl_vector(ptree &kdl_vector_json)
{
    std::vector<double> values;
    values = get_std_vector(kdl_vector_json);

    KDL::Vector kdl_vector(values[0], values[1], values[2]);

    return kdl_vector;
}

KDL::Rotation KDL::Marshaller::get_kdl_rotation(ptree &kdl_rotation_json)
{
    std::vector<double> values;
    values = get_std_vector(kdl_rotation_json);

    KDL::Rotation kdl_rotation;
    std::copy(values.begin(), values.end(), kdl_rotation.data);

    return kdl_rotation;
}

KDL::Frame KDL::Marshaller::get_kdl_frame(ptree &kdl_frame_json)
{
    KDL::Frame frame;
    frame.p = get_kdl_vector(kdl_frame_json.get_child("position"));
    frame.M = get_kdl_rotation(kdl_frame_json.get_child("rotation"));

    return frame;
}

ptree KDL::Marshaller::set_std_vector(std::vector<double> &vector)
{
    ptree vector_json;

    for(double &val : vector) {
        ptree v;
        v.put_value(val);

        vector_json.push_back(std::make_pair("", v));
    }

    return vector_json;
}

ptree KDL::Marshaller::set_kdl_vector(KDL::Vector &vector)
{
    ptree vector_json;

    ptree x, y, z;
    x.put_value(vector.x());
    y.put_value(vector.y());
    z.put_value(vector.z());

    vector_json.push_back(std::make_pair("x", x));
    vector_json.push_back(std::make_pair("y", y));
    vector_json.push_back(std::make_pair("z", z));

    return vector_json;
}

ptree KDL::Marshaller::set_kdl_rotation(KDL::Rotation &rotation)
{
    ptree rotation_json;

    for(double &val : rotation.data) {
        ptree v;
        v.put_value(val);

        rotation_json.push_back(std::make_pair("", v));
    }

    return rotation_json;
}

ptree KDL::Marshaller::set_kdl_frame(Frame &frame)
{
    ptree tr;
    tr.put_child("position", set_kdl_vector(frame.p));
    tr.put_child("rotation", set_kdl_rotation(frame.M));

    return tr;
}
