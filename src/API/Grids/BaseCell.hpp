#ifndef BASECELL_HPP
#define BASECELL_HPP

#include <libxml/parser.h>

/**
  * Base class for a Grid
  */
namespace API
{
    class BaseCell
    {
    public:
        BaseCell();
        virtual ~BaseCell();
		
		virtual bool writeToXml(xmlNodePtr cur);
		virtual bool readCellFromXml(xmlNodePtr cur);
    };
}

#endif // BASECELL_HPP
