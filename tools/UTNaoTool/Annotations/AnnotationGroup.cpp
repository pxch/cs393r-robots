#include "AnnotationGroup.h"

AnnotationGroup::AnnotationGroup(std::vector<Annotation*> annotations){
    annotations_ = annotations;
    for(uint16_t i = 0; i < annotations_.size(); i++) {
        names_[annotations_[i]->getName()] = annotations_[i];
    }
}

AnnotationGroup::AnnotationGroup(){
}

void AnnotationGroup::Serialize(YAML::Emitter& emitter) const {
    emitter << YAML::BeginDoc;
    emitter << YAML::BeginSeq;
    for(uint16_t i = 0; i < annotations_.size(); i++) {
        emitter << *annotations_[i];
    }
    emitter << YAML::EndSeq;
    emitter << YAML::EndDoc;
}

void AnnotationGroup::Deserialize(const YAML::Node& node){
    for(uint16_t i=0; i < node.size(); i++) {
        Annotation* annotation = new Annotation();
        node[i] >> *annotation;
        annotations_.push_back(annotation);
        names_[annotation->getName()] = annotation;
    }
}

std::vector<Annotation*> AnnotationGroup::getAnnotations() {
    return annotations_;
}

bool AnnotationGroup::load(Log* log) {
    std::string path = log->name;
    if(path.length() < 4) return false;
    path = path.substr(0, path.length() - 4);
    path += ".annot";
    std::ifstream fin(path.c_str());
    if(!fin.good()) return false;
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
    Deserialize(doc);
    std::cout << "Loaded annotations from " << path << "\n";
    return true;
}

bool AnnotationGroup::save(Log* log) {
    std::string path = log->name;
    return saveToLogPath(path);
}
bool AnnotationGroup::saveToLogPath(std::string logpath) {
    if(logpath.length() < 4) return false;
    std::string path = logpath.substr(0, logpath.length() - 4);
    path += ".annot";
    std::ofstream output(path.c_str());
    YAML::Emitter emitter;
    Serialize(emitter);
    output << emitter.c_str();
    output.close();
    std::cout << "Saved annotations to " << path << "\n";
    return true;
}

void AnnotationGroup::mergeAnnotations(std::vector<Annotation*> annotations, int sourceMinFrame, int targetMinFrame, int targetRange) {
    std::vector<Annotation*> toMerge;
    for(unsigned int i = 0; i < annotations.size(); i++) {
        Annotation* annotation = annotations[i];
        if(annotation->getMinFrame() <= sourceMinFrame + targetRange && annotation->getMaxFrame() >= sourceMinFrame)
            toMerge.push_back(annotation->copy());
    }
    for(unsigned int i = 0; i < toMerge.size(); i++) {
        Annotation* annotation = toMerge[i];

        // Rename collisions
        std::string orig = annotation->getName();
        int attempt = 0;
        while(names_.find(annotation->getName()) != names_.end()) {
            std::stringstream ss;
            ss << orig << "_" << attempt++;
            annotation->setName(ss.str());
        }

        // Remap frames
        int frameOffset = targetMinFrame - sourceMinFrame;
        int newMinFrame = std::max(sourceMinFrame, annotation->getMinFrame());
        int annotationRange = annotation->getMaxFrame() - annotation->getMinFrame();
        int range = std::min(annotationRange,targetRange);
        annotation->setMinFrame(newMinFrame + frameOffset);
        annotation->setMaxFrame(newMinFrame + frameOffset + range);
        annotation->remapCenterPoints(frameOffset);
        annotations_.push_back(annotation);
        names_[annotation->getName()] = annotation;
    }
}

void AnnotationGroup::deleteFromFrames(std::vector<int> frames) {
    for(unsigned int i = 0; i < frames.size(); i++) {
        int frame = frames[i];
        deleteFromFrame(frame);
    }
}

void AnnotationGroup::deleteFromFrame(int frame) {
    for(unsigned int j = 0; j < annotations_.size(); j++) {
        Annotation* annotation = annotations_[j];
        if(annotation->getMaxFrame() >= frame)
            annotation->setMaxFrame(annotation->getMaxFrame() - 1);
        if(annotation->getMinFrame() >= frame)
            annotation->setMinFrame(annotation->getMinFrame() - 1);
    }
}




