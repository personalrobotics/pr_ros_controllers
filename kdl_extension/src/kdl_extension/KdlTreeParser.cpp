#include <KdlTreeParser.h>

using namespace kdl_extension;

KdlTreeParser::KdlTreeParser()
{
}

KdlTreeParser::~KdlTreeParser()
{
}

void KdlTreeParser::loadFromFile(const std::string &fileName)
{
    if (!kdl_parser::treeFromFile(fileName, tree))
    {
        std::stringstream err;
        err << "KdlTreeParser::loadFromFile could not load file " << fileName;
        // RCS::Logger::log("gov.nasa.controllers.KdlTreeParser", log4cpp::Priority::ERROR, err.str()); TODO rm
        throw std::runtime_error(err.str());
        return;
    }

    initialize();
}

void KdlTreeParser::loadFromParam(const std::string &paramName)
{
    if (!kdl_parser::treeFromParam(paramName, tree))
    {
        std::stringstream err;
        err << "KdlTreeParser::loadFromParam could not load parameter " << paramName;
        // RCS::Logger::log("gov.nasa.controllers.KdlTreeParser", log4cpp::Priority::ERROR, err.str()); TODO rm
        throw std::runtime_error(err.str());
        return;
    }

    initialize();
}

