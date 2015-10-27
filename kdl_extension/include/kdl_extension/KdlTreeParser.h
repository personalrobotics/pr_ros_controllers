/**
 * @file KdlTreeParser.h
 * @brief Defines the KdlTreeParser class.
 * @author Ross Taylor
 * @date Sept 10, 2012
 */

#ifndef KDL_TREE_PARSER_H
#define KDL_TREE_PARSER_H

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace kdl_extension {

/**
 * @brief The KdlTreeParser class simply parses the kdl tree and stores it as a protected variable so
 * derived classes have access.
 */
class KdlTreeParser
{
    public:
        KdlTreeParser();
        virtual ~KdlTreeParser();

        void loadFromFile(const std::string& fileName);
        void loadFromParam(const std::string& paramName);

        inline void setTree(const KDL::Tree& tree_in)
        {
            tree = tree_in;
            initialize();
        }

        inline const KDL::Tree& getTree() const
        {
            return tree;
        }

    protected:
        /**
         * @brief initialize called by both of the load functions, allowing derived classes to perform
         * additional initialization after the tree is loaded
         */
        virtual void initialize()
        {
        }

        KDL::Tree tree;
};

}

#endif
