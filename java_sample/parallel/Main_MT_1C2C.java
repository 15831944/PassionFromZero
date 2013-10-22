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
 * ***** ->javac Main_MT.java    *****
 * ***** ->java Main_MT data     *****
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
import java.util.concurrent.Callable;

import java.io.File;
import java.io.FileNotFoundException;



public class Main_MT_1C2C //Product to Customer
{ 
    
    public static void main(String[] args) throws FileNotFoundException, 
                                                  InterruptedException
    {        
        long starttime= System.currentTimeMillis();
        
        // Calcute Maximum SS for each case
        Main_MT_1C2C p2c= new Main_MT_1C2C();
        p2c.calcMaximumSS(new File(args[0]));
        
        long endtime= System.currentTimeMillis();
        out.println("Running took "+(endtime-starttime)+" ms");
    }
    
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
        
        //Top countDownLatch, for all the 'goodcase'-threads
        CountDownLatch countDownLatch = new CountDownLatch(cases.size());
            
        //One 'good' case -> one thread
        for (int i=0; i<cases.size(); ++i)
        {
            //=== Check whether this case is good to go
            if ( !caseCheck(cases.get(i), caseInfo) )
            {
                out.printf("%.2f\n", result[i]);
                
                //out.println("countDownLatch.getCount()--->"+ (int)countDownLatch.getCount() );
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
                //->Multi-thread
                subThread.start();
                
                
//                //->Single-thread
//                subThread.calcSuitabilityScore();
//                result[i]=subThread.permuteSS();
//                
//                out.printf("%.2f\n", result[i]);
                
            }      
        }
           
        /*
        Long period=16L;
        if (countDownLatch.await(period, TimeUnit.SECONDS))
        {
        //    for (int i=0; i<result.length; ++i)
        //        out.printf("%.2f\n", result[i]);
        }
        else
        {
            out.println("Time is beyond "+period+" seconds.");
        }
        */
        
        /*
        countDownLatch.await();
        for (int i=0; i<result.length; ++i)
            out.printf("%.2f\n", result[i]);
        */
        
        countDownLatch.await();
    }
    
        
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
     * @param str
     * @return all the subStrings which are not ""
     */
    private static String[] analyze(String str)
    {
        ArrayList<String> output= new ArrayList<String>(10);
        
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
     * Static inner class
     * Store informations for one mask string
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
            /*
            for (int i=0; i<m; ++i)
                base.append('0');
            */
            
            countOne=0;
        }          
    }
 
    /**
     * Static inner class
     * Execution of each thread
     */
    public static class SubThread extends Thread
    {
        private final int caseIndex;
        private final String[] customers;
        private final String[] products;
        private double[] result;
        private CountDownLatch countDownLatch;
        
        private double[][] SS;
        
        public SubThread(final int caseIndex, 
                         final String[] customers, 
                         final String[] products, 
                         double[] result, 
                         CountDownLatch countDownLatch)
        {
            this.caseIndex= caseIndex;
            this.customers= customers;
            this.products= products;
            this.result=result;
            this.countDownLatch= countDownLatch;
            
            SS= new double[products.length][customers.length];
        }
        
        public void run()
        {
            try
            {        
                //=== Calculate Suitability Score (SS)
                calcSuitabilityScore();
                
                //=== Enumerate all the permutation of allocation, record the max combined SS  
                result[caseIndex]= permuteSS();

                out.printf("%.2f\n", result[caseIndex]);
            }
            catch (InterruptedException e)
            {
                        
            }
            finally
            {
                //out.println("countDownLatch.getCount()--->"+ (int)countDownLatch.getCount() );
                countDownLatch.countDown();
            }
            
        }
        
        public void calcSuitabilityScore()
        {
            int sizeOfCustomers=customers.length;
            int sizeOfProducts=products.length;
            
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

            
        }

        public double permuteSS() throws InterruptedException
        {   
            int numOfCustomers= SS[0].length;
            int numOfProducts= SS.length;

            //=== For passing reference, use an array
            //double[] maxTotalSS={0.0};
            double[] maxTotalSS= new double[]{0.0};


            //=== Perform permuting
            if (numOfCustomers == numOfProducts)
            {
                //=== Record whether or not the element in SS is used
                boolean[][] used= new boolean[numOfProducts][numOfCustomers];

                //=== Record which row is being chosen; record current sum of chosen SS
                int level=0;
                double sum=0.0;

                permutingR(SS, used, level, sum, maxTotalSS);
            }
            else if (numOfCustomers > numOfProducts)
            {
                //=== permuting according to used[][], that is set bit by bit
                CaseMoreCustomers moreC= new CaseMoreCustomers(numOfCustomers, numOfProducts, SS);
                maxTotalSS[0]=moreC.getResult();
            }
            else //Num of customers < Num of products
            {
                /*
                MaskInfo maskInfo=new MaskInfo(numOfProducts, numOfProducts-numOfCustomers);
                permutingRowDumped(0, maskInfo, SS, used, maxTotalSS);
                */
                out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            }       

            return maxTotalSS[0];
        }
   
    
    }

    
    private static class CaseMoreCustomers
    {
        private MaskInfo maskInfo;
        private final double[][] SS;
        private CountDownLatch subCountDownLatch;
        private int combinationIndex=0;
        private double[] combinationResult;
        
        private boolean[][] used;
        
        public CaseMoreCustomers(final int numOfCustomers, 
                                 final int numOfProducts, 
                                 final double[][] SS)
        {
            maskInfo=new MaskInfo(numOfCustomers, numOfCustomers-numOfProducts);
            this.SS=SS;
            this.subCountDownLatch=new CountDownLatch( combinationOf(numOfCustomers,numOfProducts) );
            
            //out.println("subCountDownLatch.getCount():"+ (int)subCountDownLatch.getCount() );
            combinationResult= new double[ (int)subCountDownLatch.getCount() ];
            //out.println("combination:"+ combinationOf(numOfCustomers,numOfProducts));
//            combinationResult= new double[ combinationOf(numOfCustomers,numOfProducts) ];
            
            used= new boolean[numOfProducts][numOfCustomers];
        }
        
        public double getResult() throws InterruptedException
        {
            permutingColDumped(0);
            
            /*
            Long timeForOneSubThread=10L;
            if (subCountDownLatch.await(timeForOneSubThread, TimeUnit.SECONDS))
            {
            //    for (int i=0; i<result.length; ++i)
            //        out.printf("%.2f\n", result[i]);
            }
            else
            {
                out.println("One thread running time is beyond "+timeForOneSubThread+" seconds.");
            }
            */
            
            
            subCountDownLatch.await();
            
            for (int i=1; i<combinationResult.length; ++i)
            {
                if (combinationResult[i]>combinationResult[0])
                    combinationResult[0]=combinationResult[i];
            }
            
            return combinationResult[0];
        }
        
        private void permutingColDumped(int pos)
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
                    
                    SubSubThread subSubThread= new SubSubThread(SS, 
                                                                used,
                                                                combinationResult,
                                                                combinationIndex,
                                                                subCountDownLatch);
                    //->Multi-thread
                    new Thread(subSubThread).start();
                    
//                    //->Single-thread
//                    subSubThread.permutingR();
//                    
//                    combinationIndex++;
                }
                else
                {
                    permutingColDumped(pos+1);
                }

                ////maskInfo.base.replace(pos, pos+1, "0");
                // Ummark this column of used[][]
                for (int row=0; row<used.length; ++row)
                    used[row][pos]=false;
            }

            maskInfo.countOne--;       
        }
        
    }
    
    
    public static class SubSubThread implements Runnable
    {
        private final double[][] SS;
        private double[] combinationResult;
        private final int combinationIndex;
        private CountDownLatch subCountDownLatch;
        private boolean[][] used_;
        
        private int level=0;
        private double sum=0;
        
        public SubSubThread(final double[][] SS, 
                            boolean[][] used,
                            double[] combinationResult,
                            final int combinationIndex,
                            CountDownLatch subCountDownLatch) 
        {
            this.SS=SS;
            this.combinationResult=combinationResult;
            this.combinationIndex=combinationIndex;
            this.subCountDownLatch=subCountDownLatch;
            
            int nRow=SS.length;
            int nCol=SS[0].length;
            used_= new boolean[nRow][nCol];
            for (int i=0; i<nRow; ++i)
                for (int j=0; j<nCol; ++j)
                    this.used_[i][j]=used[i][j];
        }
        
        public void run()
        {
            try
            {
                permutingR();
            }
            finally
            {
                subCountDownLatch.countDown();
            }
        }
        
        public void permutingR()
        {
            if (level==SS.length)
            {
                if (sum>combinationResult[combinationIndex])
                    combinationResult[combinationIndex]=sum;

                return;
            }

            for (int i=0; i<SS[0].length; ++i)
            {
                if (!used_[level][i])
                {
                    //=== Mark the related column elements below
                    for (int row=level; row<SS.length; ++row)
                        used_[row][i]=true;

                    //=== Set sum and level
                    sum+=SS[level][i];
                    level++;

                    //=== Select next SS
                    permutingR();

                    //=== Reset level and sum
                    level--;
                    sum-=SS[level][i];

                    //=== Unmark the related column elements below
                    for (int row=level; row<SS.length; ++row)
                        used_[row][i]=false;
                }           
            }

        }
        
        
    }
    
    
    
    private static int combinationOf(final int m, final int n)
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
     * Permute after some COLUMNs are dumped. Setup used[][] by
     * generating Mask string. For example: "101100" means
     * the 0th, second and 3rd column of SS are dumped
     * (counting the mask from LEFT to RIGHT)
     * @param pos
     * @param maskInfo
     * @param SS
     * @param used
     * @param maxTotalSS 
     */
    private static void permutingColDumped(int pos, 
                                          MaskInfo maskInfo,
                                          double[][] SS,
                                          boolean[][] used,
                                          double[] maxTotalSS)
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
                int level=0;
                double sum=0.0;
                permutingR(SS, used, level, sum, maxTotalSS);
                //maskInfo.combinationIndex++;
            }
            else
            {
                permutingColDumped(pos+1, maskInfo, SS, used, maxTotalSS);
            }
            
            ////maskInfo.base.replace(pos, pos+1, "0");
            // Ummark this column of used[][]
            for (int row=0; row<used.length; ++row)
                used[row][pos]=false;
        }
        
        maskInfo.countOne--;       
    }
      
    private static void permutingRowDumped(int pos, 
                                          MaskInfo maskInfo, 
                                          double[][] SS, 
                                          boolean[][] used, 
                                          double[] maxTotalSS)
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
                //PAY ATTENTION
                
                int level=0;
                double sum=0.0;
                permutingC(SS, used, level, sum, maxTotalSS);
                //maskInfo.combinationIndex++;
            }
            else
            {
                permutingRowDumped(pos+1, maskInfo, SS, used, maxTotalSS);
            }
            
            ////maskInfo.base.replace(pos, pos+1, "0");
            // Unmark this row of used[][]
            for (int col=0; col<used[0].length; ++col)
                used[pos][col]=false;
        }
        
        maskInfo.countOne--;       
    }
      
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
    
    private static int gcdOf(int num1, int num2)
    {
        if (num1<=1 || num2<=1) return 1;
        
        // Compute the great common divisor
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
