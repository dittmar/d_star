package d_star;

/**
 *
 * @author Kevin Dittmar
 */
public class Node implements Comparable
{
    //Node states
    static final int NEW = 0;
    static final int OPEN = 1;
    static final int CLOSED = 2;
    
    //Node types
    static final char UNRESTRICTED = 'O';
    static final char BLOCKED = 'B';
    static final char UNKNOWN_BLOCKED = 'U';
    static final char START = 'S';
    static final char GOAL = 'G';
    
    static final float INFINITY = 10000.0f;
    private static final int UNASSIGNED = 0;
    
    private int state;
    private char type;
    private Node backpointer;
    private float h;
    private float k;
    private final int row;
    private final int col;
    
    public Node(int row, int col, char type)
    {
        this.row = row;
        this.col = col;
        this.type = type;
        this.state = NEW;
        this.h = UNASSIGNED;
        this.k = UNASSIGNED;
        this.backpointer = null;
    }

    /**
     * Comparison for sorting nodes.  Nodes are always sorted by k value.
     * @param obj some object that should be a Node.
     * @return -1 if the provided Node is less than the calling Node,
     * 0 if the provided Node is equal to the calling Node, or 1 if
     * the provided Node is larger than the calling Node or not a Node at all.
     */
    @Override
    public int compareTo(Object obj)
    {
        if (obj instanceof Node)
        {
            Node node = (Node)obj;
            if (this.getK() < node.getK())
            {
                return -1;
            }
            else if (equals(node.getK(), this.getK()))
            {
                return 0;
            }
        }
        /* The given object isn't a node, or it has a bigger K and should
         * be put to the end of the list.
         */
        return 1;
    }
    
    /**
     * Set b(Node)
     * @param node is the Node that represents the backpointer of this Node. 
     */
    void setBackpointer(Node node)
    {
        backpointer = node;
    }
   
    /**
     * Set h(Node) to the given h value, or to INFINITY if it is >= INFINITY.
     * @param h is a float representing the current cost to reach that
     * node from the Start.
     */
    void setH(float h)
    {
        if (h >= INFINITY)
        {
            this.h = INFINITY;
        }
        else
        {
            this.h = h;
        }
    }
    
    /**
     * Set k(Node) to the given k, or to INFINITY if it is >= INFINITY.
     * @param k is a float representing the lowest cost that has been required
     * to move from Start to this node.
     */
    void setK(float k)
    {
        if (k >= INFINITY)
        {
            this.k = INFINITY;
        }
        else
        {
            this.k = k;
        }
    }
    
    /**
     * Get b(Node).
     * @return Node representing the backpointer.
     */
    Node getBackpointer()
    {
        return backpointer;
    }
    
    /**
     * @return the state of the Node.
     */
    int getState()
    {
        return state;
    }
    
    /**
     * Set the state of the Node.
     * Note:  this method expects one of the state constants defined in Node,
     * which include NEW, OPEN, and CLOSED.
     * @param state the state to set for this Node.
     */
    void setState(int state)
    {
        this.state = state;
    }
    
    /**
     * @return this Node's type, which is a constant defined in Node, including
     * START, GOAL, UNRESTRICTED, BLOCKED, or UNKNOWN_BLOCKED.
     */
    char getType()
    {
        return type;
    }
    
    /**
     * We didn't know that the node was blocked, but now we know, so update
     * this node's type to blocked.
     */
    void blockNode()
    {
        type = BLOCKED;
    }
    /**
     * Get name of Node.
     * @return String representing name of Node.
     */
    String getName()
    {
        return "(" + row + "," + col + ")";
    }
    
    /**
     * Get h(Node).
     * @return float representing h(Node).
     */
    float getH()
    {
        return h;
    }
    
    /**
     * Get k(Node).
     * @return float representing k(Node).
     */
    float getK()
    {
        return k;
    }
    
    /**
     * @return this Node's row in the world.
     */
    int getRow()
    {
        return row;
    }
    
    /**
     * @return this Node's column in the world.
     */
    int getCol()
    {
        return col;
    }
    
    /**
     * Get the description of the node, including h(Node), k(Node), b(Node),
     * and the name of the node.  Also includes the robot's location if it
     * is in this node.
     * @param robot is the Node where the robot currently is.
     * @return a String representing the node.
     */
    public String toString(Node robot)
    {
        String result = "name: " + getName() + "\n";
        result += "h: " + String.valueOf(h) + "\n";
        result += "k: " + String.valueOf(k) + "\n";
        result += "b: ";
        if (backpointer != null)
        {
            result += backpointer.getName();
        }
        result += "\n";
        result += type + "\n";
        
        if (robot == this)
        {
            result += "Robot is here.\n";
        }
        
        result += "\n";
        return result;
    }
    
    /**
     * Get the String representation of this Node without reference
     * to the robot.
     * @return String representation of this Node.
     */
    @Override
    public String toString()
    {
        return toString(null);
    }
    
    /**
     * Floats are equal if both are infinity or if they differ
     * by epsilon = .001
     * @param x is the first float to compare.
     * @param y is the second float to compare.
     * @return true if the floats are considered equal.
     */
    private boolean equals(float x, float y)
    {
        return (x >= Node.INFINITY && y >= Node.INFINITY) || 
               (Math.abs(x - y) < .0001);
    }
}
