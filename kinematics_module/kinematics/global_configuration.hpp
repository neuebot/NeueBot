#ifndef GLOBALCONFIGURATION_H
#define GLOBALCONFIGURATION_H

class GlobalConfiguration {
public:
    GlobalConfiguration(unsigned int rconf) {
        m_arm = ((rconf & 1) == 0 ? 1 : -1);
        m_elbow = ((rconf & 2) == 0 ? 1 : -1);
        m_wrist = ((rconf & 4) == 0 ? 1 : -1);
    }

    int arm() {
        return m_arm;
    }

    int elbow() {
        return m_elbow;
    }

    int wrist() {
        return m_wrist;
    }


private:
    int m_arm;
    int m_elbow;
    int m_wrist;
};

#endif //GLOBALCONFIGURATION_H
