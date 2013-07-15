#include "standalone_seg.h"

int main()
{

try
{
    CCS ccs;

    ccs.parametersReset();
    ccs.createSourceSet("reference/Segmentation/Segm_allPointsSet_.txt");
    ccs.printSourceSet();
    ccs.connectedComponentSearch();
    ccs.printSegLabel();

}
catch (const int & exc)
{
    switch (exc)
    {
    case FAILOPENSOURCEFILE:           cout << "Fail Open Source File!"          << endl; break;
    case FAILOPENSOURCESETCHECKFILE:   cout << "Fail Open SourceSet Check File!" << endl; break;
    case POINTBOUNDARYSTATEERROR:      cout << "Point Boundary State Error!"     << endl; break;
    case FAILOPENSEGLABELCHECKFILE:    cout << "Fail Open SegLabel Check File!"  << endl; break;
    }
}

    return 0;
}
