/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author bruce
 */
import static java.lang.Math.*;
import static java.lang.System.*;


interface iFigure
{
    public static final double PI=3.1415927;
    
    public abstract double calcArea();
    public abstract double calcPerimeter();
}

class Circle implements iFigure
{
    private double r=0;
    
    public Circle(double r)
    {
        this.r=r;    
    }
    
    //
    public double calcArea()
    {
        return PI*r*r;   
    }
    
    public double calcPerimeter()
    {
        return 2*PI*r;       
    }
}

class Triangle implements iFigure
{
    private double sideA=0, sideB=0;
    private double includedAngle_degree=0;
    
    public Triangle(double sideA, double sideB, double inAng)
    {
        this.sideA=sideA;
        this.sideB=sideB;
        includedAngle_degree=inAng;
    }
    
    private static double degree2radian(double degree)
    {
        return degree*PI/180.0;        
    }
    
    private static double powerOfN(double num, int N)
    {
        double temp=1.0;
        for (int i=0; i<N; ++i)
        {
            temp*=num;
        }
        
        return temp;
    }
    
    //
    public double calcArea()
    {
        return 0.5*sideA*sideB*sin( degree2radian(includedAngle_degree) );
    }
    
    public double calcPerimeter()
    {
        double angleRad= degree2radian(includedAngle_degree);
        double sideC= sqrt( powerOfN(sideA-sideB*cos(angleRad),2) 
                            + powerOfN(sideB*sin(angleRad),2) );
        return sideA+sideB+sideC;
    }
}

public class TestInterface
{
    public static void main(String[] args)
    {
        iFigure figure=new Circle(2);
        out.println("Circle Area="+figure.calcArea());
        out.println("Circle Perimeter="+figure.calcPerimeter());
        
        figure= new Triangle(1, sqrt(3), 90);
        out.println("Triangle Area="+figure.calcArea());
        out.println("Triangle Perimeter="+figure.calcPerimeter());
    }
}
