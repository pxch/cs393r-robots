#ifndef LOGSELECT_WINDOW_H
#define LOGSELECT_WINDOW_H

#include <QWidget>
#include <communications/UDPWrapper.h>

#include <QMainWindow>
#include <QCheckBox>
#include <QPushButton>
#include <QSpinBox>

class QLabel;
class QWidget;

class LogSelectWindow : public QWidget {
 Q_OBJECT

    private:
        bool log_enabled_;
        static void* listenUDP(void*);
        void listenForLoggingStatus();
        UDPWrapper* naoUDP;
        void logModeOff();
        void logModeOn();

    public:
        LogSelectWindow(QMainWindow* pa,std::vector<std::string> &block_names);

        QMainWindow* parent;

        QLabel* moduleLabels;
        QCheckBox* moduleChecks;
        int NUM_MODULES;

        QPushButton* logButton;
        QPushButton* forceStopLogButton;
        QCheckBox* batchButton;
        QPushButton* sendButton;
        QSpinBox* frameCount;
        QDoubleSpinBox* frequency;

        QLabel* groupLabels;
        QCheckBox* groupChecks;

        std::vector<std::string> block_names_;


        void sendLogMessage(const QString &msg);

        public slots:
        void sendLogSettings();
        void toggleLogEnabled();
        void stopLog();

        void locGroupToggled(bool toggle);
        void visionGroupToggled(bool toggle);
        void visionRawGroupToggled(bool toggle);
        void behaviorGroupToggled(bool toggle);
        void allGroupToggled(bool toggle);

        void updateSelectedIP(QString address);

};

#endif
