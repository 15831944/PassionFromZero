#include "standalone_seg.h"





int main()
{

try
{
    CCS ccs;
    ccs.createSourceSet("reference/Segmentation/Segm_allPointsSet_.txt");
    ccs.printSourceSet();




}
catch (const int & exc)
{
    switch (exc)
    {
    case FAILOPENSOURCEFILE:        cout << "Fail Open Source File!" << endl;       break;
    case FAILOPENOUTPUTCHECKFILE:   cout << "Fail Open Output Check File!" << endl; break;


    }


}

    return 0;
}
