/* *****************************************
 * ***** Solution of "Discount Offers" *****
 * *****************************************
 * Problem Description: https://www.codeeval.com/public_sc/48/
 * @author: Haiyue (Bruce) Li
 * @date: 09/20/2013
 * @email: jkzuoluo19@gmail.com, haiyuel@andrew.cmu.edu
 * 
 * [How To Run]
 * ********************************
 * ***** ->javac Main_MT_1C2F.java    *****
 * ***** ->java Main_MT_1C2F data     *****
 * ********************************
 * ("data": the file name of the ASCII-encoded file )
 * 
 * 
 * Copyright (c) 2013, Haiyue Li. All rights reserved.
 */


import static java.lang.System.*;
import java.lang.Math;
import java.lang.InterruptedException;

import java.util.Scanner;
import java.util.List;
//import java.util.Vector;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.ForkJoinPool;
import java.util.concurrent.RecursiveTask;
import java.util.concurrent.RecursiveAction;

import java.io.File;
import java.io.FileNotFoundException;



public class Main_MT_1C2F //2 levels multi-threads
{ 
    
    public static void main(String[] args) throws FileNotFoundException, 
                                                  InterruptedException
    {        
        long starttime= System.currentTimeMillis();
        
        // Calculate Maximum SS for each case
        Main_MT_1C2F p2c= new Main_MT_1C2F();
        p2c.calcMaximumSS(new File(args[0]));
        
        long endtime= System.currentTimeMillis();
        out.println("Running took "+(endtime-starttime)+" ms");
    }
       
    /**
     * Procedure function.  1.Calculate Maximum SS for each case of the data file.
     *                        Allocate one thread for each 'good' case. (1st level multi-threads)
     *                      2.Print the results of all the cases,
     *                        when all the threads are done. ('bad' cases print 0.00)
     * @param file: data file containing cases
     * @throws FileNotFoundException
     * @throws InterruptedException 
     */
    public void calcMaximumSS(File file) throws FileNotFoundException, 
                                                InterruptedException
    {
        Scanner scanner = new Scanner(file);
        
        //--- Read all the cases line by line
        //Vector<String> cases = new Vector<String>(50, 10);
        ArrayList<String> cases = new ArrayList<String>(50);
        String line;
        while (scanner.hasNextLine())
        {
            line=scanner.nextLine();
            // record the non-empty line
            if (line.compareTo("")!=0)
                cases.add(line);
        }
        /*        
        //DEBUG
        out.println(cases.size());
        for (int i=0; i<cases.size(); ++i)
            out.println(cases.get(i));
        */
        
        //--- For each and every case, calculate the max combined suitability score
        double[] result= new double[cases.size()];
        
        String[][] caseInfo=new String[2][]; // one row for customers
                                             // the other row for products   
        
        CountDownLatch countDownLatch = new CountDownLatch(cases.size());
            
        //One 'good' case -> one thread. here is 1st level multi-threads
        for (int i=0; i<cases.size(); ++i)
        {
            //=== Check whether this case is good to go
            if ( !caseCheck(cases.get(i), caseInfo) )
            {
                //out.printf("%.2f\n", result[i]);
                countDownLatch.countDown();
                continue;
            }                
            else
            {//This case is good to go
                String[] customers=caseInfo[0];
                String[] products=caseInfo[1];
                /*
                //DEBUG
                out.println(customers.length);
                for (String customer: customers)   
                    out.println(customer);

                out.println(products.length);
                for (String product: products)
                    out.println(product);
                */
                
                SubThread subThread= new SubThread(i, customers, products, result, countDownLatch);
                //->Multi-threads
                subThread.start();
                             
//                //->Single-thread
//                subThread.run();
//                
//                out.printf("%.2f\n", result[i]);
            }      
        }
        
        countDownLatch.await();
        for (int i=0; i<result.length; ++i)
            out.printf("%.2f\n", result[i]);
    }
   
    /**
     * Check whether the case is good or not.
     * 'Good' case means: customers and products are separated by ';'
     * customers are separated by ',' with each other, so are the products
     * @param oneCase: one line of the data file
     * @param caseInfo: the first row is used for storing customers,
     *                  the second row is used for storing products
     * @return true if the case is good; false if not
     */    
    private static boolean caseCheck(String oneCase, String[][] caseInfo)
    {
        //=== Split the case by ';'
        String[] caseSplit = oneCase.split(";");
        //CHECK
        //out.println("caseSplit.length="+caseSplit.length);
        if (caseSplit.length!=2 ||
            caseSplit[0].compareTo("")==0 || caseSplit[1].compareTo("")==0)
        {//Case format is not correct
            return false;
        }
            
        //=== analyze the case by checking ','
        caseInfo[0]=analyze(caseSplit[0]);
        caseInfo[1]=analyze(caseSplit[1]);
        //CHECK
        if (caseInfo[0][0].compareTo("")==0 ||
            caseInfo[1][0].compareTo("")==0)
        {//No valid elements
            return false;
        }
              
        return true;
    }
    
    /**
     * Split the str by ',' intelligently
     * @param str: target String
     * @return all the subStrings which is NOT ""
     */
    private static String[] analyze(String str)
    {
        ArrayList<String> output= new ArrayList<String>();
        
        int off=0;   //offset
        int next=0;        
        while ( (next=str.indexOf(',', off))!=-1 )
        {
            if (next==off)
            {//delete the ""
                off=next+1;
                continue;
            }
            
            output.add(str.substring(off, next));
            off=next+1;           
        }
        
        // No ',' at all
        if (off==0)
            return new String[]{str};
        
        // Handle the last subString
        if ( off!=str.length() )
            output.add(str.substring(off, str.length()));
        
        // No valid char at all (all of the chars are ',')   
        if (output.isEmpty())
            return new String[]{""};

       
        return output.toArray(new String[output.size()]);      
    }
  

    /**
     * Execution of each thread
     */
    private static class SubThread extends Thread
    {
        private int caseIndex;
        private String[] customers;
        private String[] products;
        private double[] result;
        private CountDownLatch countDownLatch;
        
        private SubThread(int caseIndex, String[] customers, String[] products, double[] result, CountDownLatch countDownLatch)
        {
            this.caseIndex= caseIndex;
            this.customers= customers;
            this.products= products;
            this.result=result;
            this.countDownLatch= countDownLatch;
        }
        
        public void run()
        {
            try
            {        
                //=== Calculate Suitability Score
                double[][] SS=calcSuitabilityScore(customers, products);
                
                //=== Enumerate all the permutation of allocation, record the max combined SS  
                result[caseIndex]= permuteSS(SS);
                //out.printf("%.2f\n", result[caseIndex]);
            }
            finally
            {
                countDownLatch.countDown();
            }         
        }       
    }

    /**
     * Calculate Suitability Score (SS)
     * @param customers
     * @param products
     * @return a double[][] SS, row of the array corresponds to customers,
     *         column of the array corresponds to products. For example,
     *         SS[0][3] contains the suitability score between the 1st customer
     *         and the 4th product
     */
    private static double[][] calcSuitabilityScore(final String[] customers, final String[] products)
    {
        int sizeOfCustomers=customers.length;
        int sizeOfProducts=products.length;
        
        double[][] SS= new double[sizeOfProducts][sizeOfCustomers];
        
        //=== Count the number of letters towards each customer and each product
        int[] productsLetters= new int[sizeOfProducts];
        for (int i=0; i<sizeOfProducts; ++i)
            productsLetters[i]=lettersOf(products[i]);
            
        int[] customersLetters= new int[sizeOfCustomers];
        for (int i=0; i<sizeOfCustomers; ++i)
            customersLetters[i]=lettersOf(customers[i]);
        
        //=== Algorithm stated in the problem online
        for (int i=0; i<sizeOfProducts; ++i)
        {
            if ((productsLetters[i] % 2)==0)  //even
            {
                for (int j=0; j<sizeOfCustomers; ++j)
                {
                    SS[i][j] = vowelsOf(customers[j])*1.5;
                    if ( gcdOf(productsLetters[i], customersLetters[j])>1 )
                        SS[i][j]*=1.5;
                }
            }
            else //odd
            {
                for (int j=0; j<sizeOfCustomers; ++j)
                {
                    SS[i][j] = consonantsOf(customers[j]);
                    if ( gcdOf(productsLetters[i], customersLetters[j])>1 )
                        SS[i][j]*=1.5;
                }
            }       
        }
             
        return SS;
    }
    
    
    // Setup num of threads equal to the ability of the platform
    private static final ForkJoinPool forkJoinPoll = new ForkJoinPool();
    
    /**
     * For one double[][] SS, check all the permutation 
     * (pick up only one for each row, and only one for each column),
     * @param SS
     * @return the maximum combined(sum of) suitability score
     */
    private static double permuteSS(double[][] SS)
    {   
        int numOfCustomers= SS[0].length;
        int numOfProducts= SS.length;
   
        //Setup ForkJoinTasks, here is 2nd level multi-threads  
        TargetCase tc=new TargetCase(numOfCustomers, numOfProducts, SS);      
        forkJoinPoll.invoke(tc);            
            
        return tc.getResult();
    }

    /**
     * ForkJoinTask
     */
    private static class TargetCase extends RecursiveAction
    {
        private final byte caseType; //0: c/p equal
                                     //1: more customers
                                     //2: more products
        private final MaskInfo maskInfo;
        private final double[][] SS;
        private List< RecursiveTask<Double> > taskList;     
        private boolean[][] used;
        
        private double result=0.0;
        
        private TargetCase(final int numOfCustomers, 
                           final int numOfProducts,
                           final double[][] SS)
        {
            if (numOfCustomers==numOfProducts)
            {
                caseType=0;
                maskInfo=null;
            }
            else if (numOfCustomers>numOfProducts)
            {
                caseType=1;
                maskInfo=new MaskInfo(numOfCustomers, numOfCustomers-numOfProducts);          
            }
            else
            {
                caseType=2;
                maskInfo=new MaskInfo(numOfProducts, numOfProducts-numOfCustomers);          
            }
            
            this.SS=SS;
            
            taskList= new LinkedList<>();
            used= new boolean[numOfProducts][numOfCustomers];
        }
        
        protected void compute()
        {
            if (caseType==0) //c/p equal, no need to create child task
            {
                double[] maxTotalSS={0.0};
                permutingR(SS, used, 0, 0.0, maxTotalSS);
                result=maxTotalSS[0];
            }
            else if (caseType==1) //more customers
                setUsed_colDumped(0);
            else
                setUsed_rowDumped(0);
        }
        
        private double getResult()
        {
            if (caseType==0) return result;
            
            double taskValue;
            //out.println("SS.length,SS[0].length,List_size: "+ SS.length+","+ SS[0].length+","+taskList.size());         
            for (RecursiveTask<Double> task: taskList)
            {
                if ((taskValue=task.join()) > result)
                    result=taskValue;                
            }
            
            return result;
        }
        
        /* Permute after some COLUMNs are dumped. Setup used[][] by
         * generating Mask string. For example: "101100" means
         * the 0th, second and 3rd column of SS are dumped
         * (counting the mask from LEFT to RIGHT)
         */
        private void setUsed_colDumped(int pos)
        {   
            maskInfo.countOne++;
            for (; pos<maskInfo.totalBits; ++pos)
            {
                ////maskInfo.base.replace(pos, pos+1, "1");
                // Mark this column of used[][]
                for (int row=0; row<used.length; ++row)
                    used[row][pos]=true;

                if (maskInfo.countOne == maskInfo.bitsOfOne)
                {
                    PermutingTask task= new PermutingTask(SS, used, caseType);
                    taskList.add(task);
                    task.fork();
                }
                else
                {
                    setUsed_colDumped(pos+1);
                }

                ////maskInfo.base.replace(pos, pos+1, "0");
                // Ummark this column of used[][]
                for (int row=0; row<used.length; ++row)
                    used[row][pos]=false;
            }

            maskInfo.countOne--;       
        }
    
        private void setUsed_rowDumped(int pos)
        {
            maskInfo.countOne++;
            for (; pos<maskInfo.totalBits; ++pos)
            {
                ////maskInfo.base.replace(pos, pos+1, "1");
                // Mark this row of used[][]
                for (int col=0; col<used[0].length; ++col)
                    used[pos][col]=true;

                if (maskInfo.countOne == maskInfo.bitsOfOne)
                {                 
                    PermutingTask task= new PermutingTask(SS, used, caseType);
                    taskList.add(task);
                    task.fork();
                }
                else
                {
                    setUsed_rowDumped(pos+1);
                }

                ////maskInfo.base.replace(pos, pos+1, "0");
                // Unmark this row of used[][]
                for (int col=0; col<used[0].length; ++col)
                    used[pos][col]=false;
            }

            maskInfo.countOne--;       
        }
        
       
    }
    
    
    /**
     * Child ForkJoinTask
     */
    private static class PermutingTask extends RecursiveTask<Double>
    {
        private final byte caseType;
        private final double[][] SS;   
        private boolean[][] used_;
        
        private PermutingTask(final double[][] SS, boolean[][] used, final byte caseType)
        {
            this.caseType=caseType;
            this.SS=SS;
            
            int nRow=used.length;
            int nCol=used[0].length;
            this.used_= new boolean[nRow][nCol];
            for (int i=0; i<nRow; ++i)
                for (int j=0; j<nCol; ++j)
                    this.used_[i][j]=used[i][j];
        }
             
        protected Double compute()
        {
            double[] maxTotalSS={0.0};
            
            if (caseType==1)
                permutingR(SS, used_, 0, 0.0, maxTotalSS);
            else
                permutingC(SS, used_, 0, 0.0, maxTotalSS);
            
            return maxTotalSS[0];
        }
    }
    
    
    /**
     * Traverse all the situations of combined SS, picking up 
     * elements one row by one row, record the max one
     * @param flatMatrix: 'flat' means matrix[0].length >= matrix.length
     * @param used: whether this SS has been chosen
     * @param level: which row of the SS[][] is being handled
     * @param sum: sum of SSs that have been chosen
     * @param maxTotalSS: output. The max total(combined) SS 
     */
    private static void permutingR(final double[][] flatMatrix, 
                                   boolean[][] used__,
                                   int level,
                                   double sum,
                                   double[] maxTotalSS)
    {
        if (level==flatMatrix.length)
        {
            if (sum>maxTotalSS[0])
                maxTotalSS[0]=sum;
            
            return;
        }
        
        for (int i=0; i<flatMatrix[0].length; ++i)
        {
            if (!used__[level][i])
            {
                //=== Mark the related column elements below
                for (int row=level; row<flatMatrix.length; ++row)
                    used__[row][i]=true;
                
                //=== Set sum and level
                sum+=flatMatrix[level][i];
                level++;
                
                //=== Select next SS
                permutingR(flatMatrix, used__, level, sum, maxTotalSS);
                
                //=== Reset level and sum
                level--;
                sum-=flatMatrix[level][i];
                
                //=== Unmark the related column elements below
                for (int row=level; row<flatMatrix.length; ++row)
                    used__[row][i]=false;
            }           
        }
  
    }
    
    /**
     * Traverse all the situations of combined SS, picking up
     * elements one column by one column, record the max one
     * @param tallMatrix: 'tall' means matrix.length >= matrix[0].length
     * @param used: whether this SS has been chosen
     * @param level: which col of the SS[][] is being handled
     * @param sum: sum of SSs that have been chosen
     * @param maxTotalSS: output. The max total(combined) SS 
     */
    private static void permutingC(final double[][] tallMatrix, 
                                   boolean[][] used,
                                   int level,
                                   double sum,
                                   double[] maxTotalSS)
    {
        if (level==tallMatrix[0].length)
        {
            if (sum>maxTotalSS[0])
                maxTotalSS[0]=sum;
            
            return;
        }
        
        for (int i=0; i<tallMatrix.length; ++i)
        {
            if (!used[i][level])
            {
                //=== Mark the related row elements to the right
                for (int col=level; col<tallMatrix[0].length; ++col)
                    used[i][col]=true;
                
                //=== Set sum and level
                sum+=tallMatrix[i][level];
                level++;
                
                //=== Select next SS
                permutingC(tallMatrix, used, level, sum, maxTotalSS);
                
                //=== Reset level and sum
                level--;
                sum-=tallMatrix[i][level];
                
                //=== Unmark the related row elements to the right
                for (int col=level; col<tallMatrix[0].length; ++col)
                    used[i][col]=false;
            }           
        }
  
    }
    
          
    /**
     * Store informations for one mask string (for example: "010011")
     */
    private static class MaskInfo
    {
        private int totalBits;
        private int bitsOfOne;
        private int countOne;
        //private int combinationIndex=0;
        //private StringBuffer base= new StringBuffer();
        
        MaskInfo(final int m, final int n)
        {
            totalBits=m;
            bitsOfOne=n;
            countOne=0;
            /*
            for (int i=0; i<m; ++i)
                base.append('0');
            */         
        }          
    }

    /**
     * Compute the letters of a String
     * @param str
     * @return the number of letters contained
     */  
    private static int lettersOf(String str)
    {
        int letters=0;
        
        /*
        // Include all the characters except SPACE
        String[] substrs=str.split(" ");     
        for (String substr: substrs)
            letters+=substr.length();
        */
        
        // Include ONLY letters
        for (int i=0; i<str.length(); ++i)
        {
            if ( ('A'<=str.charAt(i) && str.charAt(i)<='Z') ||
                 ('a'<=str.charAt(i) && str.charAt(i)<='z') )
            {
                letters++;
            }       
        }
        
        return letters;
    }
    
    /**
     * Compute the vowels of a String
     * @param str
     * @return the number of vowels contained
     */
    private static int vowelsOf(String str)
    {
        int vowels=0;        
        for (int i=0; i<str.length(); ++i)
        {
            if ( str.charAt(i) == 'a' || str.charAt(i) == 'A' ||
                 str.charAt(i) == 'e' || str.charAt(i) == 'E' ||
                 str.charAt(i) == 'i' || str.charAt(i) == 'I' ||
                 str.charAt(i) == 'o' || str.charAt(i) == 'O' ||
                 str.charAt(i) == 'u' || str.charAt(i) == 'U' ||
                 str.charAt(i) == 'y' )//|| str.charAt(i) == 'Y' )
            {
                vowels++;
            }         
        }
        
        return vowels;
    }
    
    /**
     * Compute the consonants of a String
     * @param str
     * @return the number of consonants contained
     */
    private static int consonantsOf(String str)
    {
        int consonants=0;        
        for (int i=0; i<str.length(); ++i)
        {
            if ( ('A'<=str.charAt(i) && str.charAt(i)<='Z') ||
                 ('a'<=str.charAt(i) && str.charAt(i)<='z') )
            {
                if ( str.charAt(i) == 'a' || str.charAt(i) == 'A' ||
                     str.charAt(i) == 'e' || str.charAt(i) == 'E' ||
                     str.charAt(i) == 'i' || str.charAt(i) == 'I' ||
                     str.charAt(i) == 'o' || str.charAt(i) == 'O' ||
                     str.charAt(i) == 'u' || str.charAt(i) == 'U' ||
                     str.charAt(i) == 'y' )//|| str.charAt(i) == 'Y' )
                {}
                else
                    consonants++;
            }
        }
        
        return consonants;     
    }
    
    /**
     * Compute the great common divisor(gcd) of two positive integers
     * (Euclidean algorithm)
     * @param num1
     * @param num2
     * @return the gcd
     */
    private static int gcdOf(int num1, int num2)
    {
        if (num1<=1 || num2<=1) return 1;
        
        // Euclidean algorithm
        int minValue=num1, difference=num2;
        while (minValue != difference)
        {
            num1=minValue;
            num2=difference;
            minValue=Math.min(num1, num2);
            difference=Math.abs(num1 - num2);
        }
        
        return minValue; // Here, minValue==difference
    }
      
}
