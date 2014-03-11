#ifndef BASECELL_HPP
#define BASECELL_HPP

#include <libxml/parser.h>

/**
 * Base class for a Grid
 */
namespace Move3D
{
class BaseCell
{
public:
    BaseCell();
    virtual ~BaseCell();

    virtual void draw() = 0;

    virtual bool writeToXml(xmlNodePtr cur);
    virtual bool readCellFromXml(xmlNodePtr cur);
};
}

#endif // BASECELL_HPP
