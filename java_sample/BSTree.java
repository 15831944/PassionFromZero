
import static java.lang.System.*;


class Node
{
    private int value;
    
    public Node left;
    public Node right;
    public Node parent;
    
    public Node()
    {
        value=0;
        
        left=null;
        right=null;
        parent=null;
    }
    
    public Node(int value)
    {
        this.value=value;
        
        left=null;
        right=null;
        parent=null;
    }
    
    public int value()
    {
        return value;
    }
     
}



public class BSTree 
{
    private Node root;
    
    public BSTree()
    {
        root=null;      
    }
    
    public BSTree(Node root)
    {
        this.root=root;        
    }
    
    //ADD
    public Node addNode(Node node)
    {
        if (root==null)
            return root=node;   
        
        Node seeker=root;
        while(true)
        {
            if (node.value()<=seeker.value())
            {
                if (seeker.left==null)
                {
                    seeker.left=node;
                    node.parent=seeker;
                    break;
                }
                else
                    seeker=seeker.left;
            }
            else
            {
                if (seeker.right==null)
                {
                    seeker.right=node;
                    node.parent=seeker;
                    break;
                }
                else
                    seeker=seeker.right;
            }       
        }
        
        return root;
    }
    
    //Delete
    //From root, delete the first node whose value==value, and the subtree below
    public boolean deleteNode(int value)
    {
        Node result=findNode(root, value);
        
        if (result==null) return false;
        
        if (result==root)
        {
            root=null;
            return true;
        }
        
        Node parentNode=result.parent;
        if (parentNode.left==result)
            parentNode.left=null;
        else
            parentNode.right=null;
        
        result.parent=null;
        return true;
    }
    
    //Find
    public Node findNode(Node seeker, int value)
    {
        if (seeker==null) return null;
        
        if (seeker.value()==value)
        {
            return seeker;
        }
        else if (seeker.value()<value)
        {
            return findNode(seeker.right, value);
        }
        else
        {
            return findNode(seeker.left, value);
        }
    }
    
    
    //Traverse
    public void inorderPrint(Node node)
    {
        if (root==null)
            out.println("Empty BSTree!");
        
        if (node==null) return;
        
        inorderPrint(node.left); 
        out.println(node.value());
        inorderPrint(node.right);       
    }
    
    public static void main(String[] args)
    {
        BSTree bst=new BSTree();
        
        //Add nodes to create a sample tree
        bst.addNode(new Node(10));
        bst.addNode(new Node(5));
        bst.addNode(new Node(13));
        bst.addNode(new Node(1));
        bst.addNode(new Node(7));
        bst.addNode(new Node(15));
        bst.addNode(new Node(0));
        bst.addNode(new Node(3));
              
        //Print all nodes of the tree
        bst.inorderPrint(bst.root);
        
        //Test
        out.println("~~~~~~~~~~~~~~~~");
        out.println( bst.findNode(bst.root, 7) );
        
        out.println(bst.deleteNode(10));
        bst.inorderPrint(bst.root);
    }
    
    
}
