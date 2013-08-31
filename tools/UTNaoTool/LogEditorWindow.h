#ifndef LOGEDITOR_WINDOW_H
#define LOGEDITOR_WINDOW_H

#include <vector>
#include <map>
#include <sstream>

#include <memory/Logger.h>
#include <memory/LogReader.h>
#include "ui_LogEditorWindow.h"
#include "Annotations/AnnotationGroup.h"

class FrameListWidgetItem : public QListWidgetItem {
    private:
        int frame_;
        std::string logfile_;
        void init(std::string logfile, int frame){
            frame_ = frame;
            logfile_ = logfile;
            std::string display = logfile;
            if(display.length() > 4)
                display = display.substr(0,display.length() - 4);
            std::stringstream ss;
            ss << display << ": " << frame;
            setText(QString::fromStdString(ss.str()));
        }
    public:
        FrameListWidgetItem(std::string logfile, int frame) {
            init(logfile,frame);
        }
        FrameListWidgetItem(const FrameListWidgetItem& other) : QListWidgetItem(other) {
            init(other.logfile_, other.frame_);
        }
        std::string getFile() {
            return logfile_;
        }
        int getFrame() {
            return frame_;
        }
};

class LogEditorWindow : public QMainWindow, public Ui_LogEditorWindow {
 Q_OBJECT

    private:
        bool textUpdating_;
        Log* log_;
        std::map<std::string, Log* > loaded_logs_;
        Log* lookupLog(std::string);
        AnnotationGroup* lookupGroup(Log*);
        AnnotationGroup *new_group_, *loaded_group_;
        std::map<Log*, AnnotationGroup*> log_annotations_;

    public:
        LogEditorWindow(QMainWindow* pa);
        Logger *logWriter;
        QMainWindow* parent;
        void keyPressEvent(QKeyEvent *event);

    signals:
        void prevFrame();
        void nextFrame();

    public slots:
        void loadLogs();
        void combineAllLogs();
        void addAllFrames();
        void addFrames();
        void delFrames();
        void clearFrames();

        void saveLog();
        void closeLog();

        void addShot(QListWidgetItem* item);
        void removeShot(QListWidgetItem* item);
        void lognameChanged(const QString&);
        void loadLog(QListWidgetItem* item);

        void handleItemSelectionChanged();
};

#endif

