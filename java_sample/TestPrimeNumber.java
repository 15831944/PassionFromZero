

/**
 * Optimized sieve of Eratosthenes
 * 
 * 
 * @author bruce
 */

import static java.lang.System.*;
import java.util.LinkedList;
import java.util.ListIterator;

public class TestPrimeNumber 
{
    public static void main(String[] args)
    {
        long starttime= System.currentTimeMillis();
        
        generatePrimeNumbers(Integer.parseInt(args[0]));
            
        long endtime= System.currentTimeMillis();
        out.println("Running took "+(endtime-starttime)+" ms");
    }
    
    private static final int[] known={2, 3, 5, 7, 11, 13, 17, 19};
    private static int count=0;
    
    private static void generatePrimeNumbers(int limit)
    {
        if (limit<known[0]) 
        {
            out.println("no prime number found below: " + limit);
            return;
        }
        
        if (limit<=known[known.length-1])
        {
            for (int i=0; i<known.length; ++i)
            {
                if (known[i]<=limit)
                    {out.print(known[i]+",\t"); count++;}
                else
                    break;
            }          
        }
        else
        {
            //===Algorithm starts
            //First, print 2
            out.print(2+",\t");

            //Second, generate a linklist of consecutive odds from 3 to (limit - 1or0)
            LinkedList<Integer> oddList= new LinkedList<Integer>();
            int odd=3;
            while (odd<=limit)
            {
                oddList.add(new Integer(odd));
                odd+=2;  
            }
            
            //DEBUG
//            ListIterator<Integer> itr=oddList.listIterator(0);
//            while(itr.hasNext())
//                out.print(itr.next()+",\t");
            
            
            //Third, pick up the first unmarked(undeleted) number,
            //record it (call it p), then mark(delete) those 
            //numbers at interval: 2p, (starting from p^2).
            //When p^2 > limit, stop
            int p=oddList.getFirst();
            int index=0, pos=index;
            Integer multiple=p;
            while(p*p<=limit)
            {
                while( (multiple+=2*p) <= limit )
                {
                    oddList.remove(multiple);
                    //pos=removeFrom(oddList, multiple, pos);                
                }
                
                p=oddList.get(++index);
                multiple=p;
                //pos=index;
            }
     
            //Fourth, all the last numbers in the list are prime numbers
            ListIterator<Integer> traverse=oddList.listIterator(0);
            //ListIterator<Integer> traverse=oddList.new ListItr(0); // Wrong: ListItr has private access
            //ListIterator<Integer> traverse=new LinkedList.ListItr(0); // Same Wrong: ListItr has private access
            while(traverse.hasNext())
                out.print(traverse.next()+",\t");

            count=oddList.size()+1;
        }
        
        out.println("\nTotally "+ count +" prime numbers.");    
    }
    
    /**
     * 
     * @param num
     * @return the pos of the removed node OR the first node that is
     *         bigger than num
     */
    private static int removeFrom(LinkedList<Integer> oddList, Integer num, int pos)
    {
        if (pos>=oddList.size() || num>oddList.getLast()) return -1;

        
        while (oddList.get(pos)<num)
            pos++;
     
        if (oddList.get(pos)==num.intValue())
            oddList.remove(pos);

        return pos;
    }
    
}
