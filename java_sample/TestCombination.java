class num
{
    int[] num;
    int count=0;
    int m; //1
    int n; //0
    int total=0;
    
    int tempRecord=-1;
    
    num(int m,int n)
    {
        this.m=m;
        this.n=n;
        num=new int[m+n];
        
        for (int a=0;a<m+n;a++)
        {
            num[a]=0;
          
        }
      
    }
    
    void perform()
    {
        if (m==0)
        {
            for (int i=0; i<n; ++i)
                System.out.print(0);
            
            System.out.println(";");
            total++;
        }
        else
        {
            print();
        }
    }
    
    void perform(int pos)
    {
        if (m==0)
        {
            for (int i=0; i<n; ++i)
                System.out.print(0);
            
            System.out.println(";");
            total++;
        }
        else
        {
            print(pos);
        }
    }
    
    void print()
    {
        int temp=tempRecord+1;
        
        count++;
        for ( ;temp<m+n;temp++ )
        {
            num[ temp ]=1;
            if (count==m)
            {
                for (int a = 0;a<m+n;a++)
                {
                    System.out.print(num[a]);
                }
                System.out.println(";");
                total++;
            }
            else
            {
                tempRecord=temp;
                print();
                
            }
            num[ temp ] = 0;
        }
        count--;
        
        return;
    }
    
    void print(int temp)
    {   
        count++;
        for ( ;temp<m+n;temp++ )
        {
            num[ temp ]=1;
            if (count==m)
            {
                for (int a = 0;a<m+n;a++)
                {
                    System.out.print(num[a]);
                }
                System.out.println(";");
                total++;
            }
            else
            {
                print(temp+1);
                
            }
            num[ temp ] = 0;
        }
        count--;
        
        return;
    }

    public static int combinationOf(final int m, final int n)
    {    
        // Wrong case
        if ( m<1 || n<0 || m<n ) return 0;
        
        // Easy case
        if (m==n || n==0) return 1;
        
        double p=1.0;
        for (int i=0; i<n; ++i)
            p*=(m-i)/(double)(n-i);
        
        return (int)p;
    }


}

public class TestCombination
{
    public static void main(String[] args)
    {
        int m = Integer.parseInt(args[0]);
        int n = Integer.parseInt(args[1]);
        
        num t=new num(m,n);
        t.perform();       
        System.out.println("total="+t.total);
        
        System.out.println();
        
        num t2=new num(m,n);
        t2.perform(0);       
        System.out.println("total="+t2.total);
        
        System.out.println("C("+(m+n)+","+m+")="+ num.combinationOf(m+n,m));
        
    }
}