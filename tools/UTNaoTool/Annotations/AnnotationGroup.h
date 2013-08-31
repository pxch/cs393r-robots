#ifndef ANNOTATION_GROUP_H
#define ANNOTATION_GROUP_H

#include "yaml-cpp/yaml.h"
#include "Annotation.h"
#include <memory/Log.h>
#include <fstream>
#include <map>
#include <sstream>
#include <stdint.h>

class AnnotationGroup {

    private:
        std::vector<Annotation*> annotations_;
        std::map<std::string,Annotation*> names_;

    public:
        AnnotationGroup();
        AnnotationGroup(std::vector<Annotation*>);
        void Serialize(YAML::Emitter&) const;
        void Deserialize(const YAML::Node&);
        std::vector<Annotation*> getAnnotations();
        bool save(Log*);
        bool saveToLogPath(std::string);
        bool load(Log*);
        void mergeAnnotations(std::vector<Annotation*>,int,int,int);
        void deleteFromFrames(std::vector<int>);
        void deleteFromFrame(int);
};

#endif
