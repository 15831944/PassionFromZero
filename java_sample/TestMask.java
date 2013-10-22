
//import java.util.ArrayList;
//import java.util.Arrays;
import java.util.Vector;
        
public class TestMask
{
	public int getOneBit(int num, int n)
	{
		int i = 1<<(n-1);
		int nb = num & i;
		return nb >> (n-1);
	}

	public static void main(String[] args)
	{
		int x=9;
		TestMask objmask= new TestMask();
                System.out.println("4th bit:"+objmask.getOneBit(x,4));
                System.out.printf("3th bit:%d\n",objmask.getOneBit(x,3));
                System.out.print("2nd bit:"+objmask.getOneBit(x,2));
                System.out.println("\n1st bit:"+objmask.getOneBit(x,1));
                System.out.println(objmask.toString());
                System.out.println(objmask);
        }


}


