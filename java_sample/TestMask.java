
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
		TestMask obj= new TestMask();
		System.out.println(obj.getOneBit(x,4));
		System.out.println(obj.getOneBit(x,3));
		System.out.println(obj.getOneBit(x,2));
		System.out.println(obj.getOneBit(x,1));
	}


}


