
#include "standalone_seg.h"
#include <iomanip>


void CCS::createSourceSet(const char* filename)
{
    ifstream Ain(filename, ios::in);
    if (!Ain.is_open()) throw FAILOPENSOURCEFILE;

    int i;
    string str;
    RecPoint3D p;

    // read 2 lines first
    for (i=0; i<2; ++i)  // 2, 682, 1368
        getline(Ain, str);

    // then, begin reading points
    while (1)
    {
        getline(Ain, str, '\t');
        if ( !(p.x=atof(str.c_str())) ) break;
        getline(Ain, str, '\t');
        p.y=atof(str.c_str());
        getline(Ain, str, '\n');
        p.z=atof(str.c_str());

        sourceSet.insert(p);
    }

    Ain.close();
    return ;
}

void CCS::printSourceSet()
{
    ofstream Aout;
    Aout.open("SourceSet_Check.txt", ios::out|ios::trunc);
    if ( !Aout.is_open() ) throw FAILOPENOUTPUTCHECKFILE;

    Aout << "PointsNum: " << sourceSet.size() << " ----------------------------------------" << endl;
    for (set<RecPoint3D,ltpt3D>::const_iterator itr = sourceSet.begin(); itr != sourceSet.end(); ++itr)
        Aout << fixed << setprecision(4) << itr->x << " " << itr->y << "    " << itr->z << endl;

    Aout << "------------------------------------------------------------------------------" << endl;
    Aout.close();
    return ;

}
