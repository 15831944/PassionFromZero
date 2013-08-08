#include "standalone_seg.h"

int main()
{

try
{
    CCS ccs;

    ccs.parametersReset();
    ccs.centerMap("reference/Segmentation/Segm_vehiclePose_.txt");
    ccs.createSourceSet("reference/Segmentation/Segm_cellCentrePointsSet_.txt");
    ccs.printSourceSet();
    ccs.cellSweepSegment();
    ccs.printSegLabel();

}
catch (const int & exc)
{
    switch (exc)
    {
    case FAILOPENSOURCESETFILE:        cout << "Fail Open Source Set File!"      << endl; break;
    case FAILOPENSOURCESETCHECKFILE:   cout << "Fail Open SourceSet Check File!" << endl; break;
    case FAILOPENVEHICLEPOSEFILE:      cout << "Fail Open Vehicle Pose File!"    << endl; break;
    case FAILOPENSEGLABELCHECKFILE:    cout << "Fail Open SegLabel Check File!"  << endl; break;
    }
}

    return 0;
}
