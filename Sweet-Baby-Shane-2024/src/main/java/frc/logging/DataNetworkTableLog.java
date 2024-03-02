package frc.logging;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;

public class DataNetworkTableLog
{
    public enum COLUMN_TYPE{
        BYTE,
        SHORT,
        INTEGER,
        LONG,
        FLOAT,
        DOUBLE,
        STRING,
        LONG_ARRAY,
        FLOAT_ARRAY,
        DOUBLE_ARRAY,
        STRING_ARRAY,
        OTHER
    }

    public static final String COLUMN_PUBLISHER_NAME_FORMAT = "%s/%s";

    private NetworkTableInstance networkTableInstance;
    private String               tableName;

    private Map< String, Publisher > publishers = new HashMap< String, Publisher >();

    public DataNetworkTableLog( String tableName, Map< String, COLUMN_TYPE > columns )
    {
        networkTableInstance = NetworkTableInstance.getDefault();
    
        this.tableName = tableName.replaceAll( "\\.", "/" );

        if ( columns != null )
        {
            createColumns( columns );
        }
    }

    public DataNetworkTableLog( String tableName )
    {
        this( tableName, null );
    }

    public void createColumns( Map< String, COLUMN_TYPE > columns )
    {

        columns.forEach( ( name, type ) -> {

            switch( type )
            {
                case BYTE:
                case SHORT:
                case INTEGER:
                case LONG:
                {
                    Publisher pub = publishers.get( name );
                    if ( pub == null )
                    {
                        pub = networkTableInstance.getIntegerTopic(
                            String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                        publishers.put( name, pub );
                    }
                }
                    break;
                case FLOAT:
                {
                    Publisher pub = publishers.get( name );
                    if ( pub == null )
                    {
                        pub = networkTableInstance.getFloatTopic(
                            String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                        publishers.put( name, pub );
                    }
                }
                    break;
                case DOUBLE:
                {
                    Publisher pub = publishers.get( name );
                    if ( pub == null )
                    {
                        pub = networkTableInstance.getDoubleTopic(
                            String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                        publishers.put( name, pub );
                    }
                }
                    break;
                case STRING:
                {
                    Publisher pub = publishers.get( name );
                    if ( pub == null )
                    {
                        pub = networkTableInstance.getStringTopic(
                            String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                        publishers.put( name, pub );
                    }
                }
                    break;
                case LONG_ARRAY:
                {
                    Publisher pub = publishers.get( name );
                    if ( pub == null )
                    {
                        pub = networkTableInstance.getIntegerArrayTopic(
                            String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                        publishers.put( name, pub );
                    }
                }
                    break;
                case FLOAT_ARRAY:
                {
                    Publisher pub = publishers.get( name );
                    if ( pub == null )
                    {
                        pub = networkTableInstance.getIntegerArrayTopic(
                            String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                        publishers.put( name, pub );
                    }
                }
                    break;
                case DOUBLE_ARRAY:
                {
                    Publisher pub = publishers.get( name );
                    if ( pub == null )
                    {
                        pub = networkTableInstance.getIntegerArrayTopic(
                            String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                        publishers.put( name, pub );
                    }
                }
                    break;
                case STRING_ARRAY:
                {
                    Publisher pub = publishers.get( name );
                    if ( pub == null )
                    {
                        pub = networkTableInstance.getIntegerArrayTopic(
                            String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                        publishers.put( name, pub );
                    }
                }
                    break;
                case OTHER:
                {
                    Publisher pub = publishers.get( name );
                    if ( pub == null )
                    {
                        pub = networkTableInstance.getStringTopic(
                            String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                        publishers.put( name, pub );
                    }
                }
                    break;
            }

        } );
    }

    public void publish( String name, Object obj )
    {

        if ( ( obj instanceof Long    ) ||
             ( obj instanceof Integer ) ||
             ( obj instanceof Short   ) ||
             ( obj instanceof Byte    )    )
        {

            Publisher pub = publishers.get( name );
            if ( pub == null )
            {
                pub = networkTableInstance.getIntegerTopic(
                    String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                publishers.put( name, pub );
            }

            if ( pub instanceof IntegerPublisher )
            {
                ( (IntegerPublisher) pub ).set( (Long) obj );
            }

        } else
        if ( obj instanceof Float )
        {

            Publisher pub = publishers.get( name );
            if ( pub == null )
            {
                pub = networkTableInstance.getFloatTopic(
                    String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                publishers.put( name, pub );
            }

            if ( pub instanceof FloatPublisher )
            {
                ( (FloatPublisher) pub ).set( (Float) obj );
            }

        } else
        if ( obj instanceof Double )
        {

            Publisher pub = publishers.get( name );
            if ( pub == null )
            {
                pub = networkTableInstance.getDoubleTopic(
                    String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                publishers.put( name, pub );
            }

            if ( pub instanceof DoublePublisher )
            {
                ( (DoublePublisher) pub ).set( (Double) obj );
            }

        } else
        if ( obj instanceof String )
        {

            Publisher pub = publishers.get( name );
            if ( pub == null )
            {
                pub = networkTableInstance.getStringTopic(
                    String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                publishers.put( name, pub );
            }

            if ( pub instanceof StringPublisher )
            {
                ( (StringPublisher) pub ).set( (String) obj );
            }

        } else
        if ( obj instanceof long[] )
        {

            Publisher pub = publishers.get( name );
            if ( pub == null )
            {
                pub = networkTableInstance.getIntegerArrayTopic(
                    String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                publishers.put( name, pub );
            }

            if ( pub instanceof IntegerArrayPublisher )
            {
                ( (IntegerArrayPublisher) pub ).set( (long[]) obj );
            }

        } else
        if ( obj instanceof float[] )
        {

            Publisher pub = publishers.get( name );
            if ( pub == null )
            {
                pub = networkTableInstance.getIntegerArrayTopic(
                    String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                publishers.put( name, pub );
            }

            if ( pub instanceof FloatArrayPublisher )
            {
                ( (FloatArrayPublisher) pub ).set( (float[]) obj );
            }

        } else
        if ( obj instanceof double[] )
        {

            Publisher pub = publishers.get( name );
            if ( pub == null )
            {
                pub = networkTableInstance.getIntegerArrayTopic(
                    String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                publishers.put( name, pub );
            }

            if ( pub instanceof DoubleArrayPublisher )
            {
                ( (DoubleArrayPublisher) pub ).set( (double[]) obj );
            }

        } else
        if ( obj instanceof String[] )
        {

            Publisher pub = publishers.get( name );
            if ( pub == null )
            {
                pub = networkTableInstance.getIntegerArrayTopic(
                    String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                publishers.put( name, pub );
            }

            if ( pub instanceof StringArrayPublisher )
            {
                ( (StringArrayPublisher) pub ).set( (String[]) obj );
            }

        } else {

            Publisher pub = publishers.get( name );
            if ( pub == null )
            {
                pub = networkTableInstance.getStringTopic(
                    String.format( COLUMN_PUBLISHER_NAME_FORMAT, tableName, name ) ).publish();
                publishers.put( name, pub );
            }

            if ( pub instanceof StringPublisher )
            {
                ( (StringPublisher) pub ).set( obj.toString() );
            }

        }

    }

}
