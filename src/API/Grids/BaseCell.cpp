#include "BaseCell.hpp"

using namespace Move3D;

BaseCell::BaseCell()
{
}

BaseCell::~BaseCell()
{
}

bool BaseCell::readCellFromXml(xmlNodePtr cur)
{
	return false;
}

bool BaseCell::writeToXml(xmlNodePtr cur)
{
	return false;
}