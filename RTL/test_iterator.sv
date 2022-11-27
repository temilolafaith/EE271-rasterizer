/*
 *  Bounding Box Sample Test Iteration
 *
 *  Inputs:
 *    BBox and triangle Information
 *
 *  Outputs:
 *    Subsample location and triangle Information
 *
 *  Function:
 *    Iterate from left to right bottom to top
 *    across the bounding box.
 *
 *    While iterating set the halt signal in
 *    order to hold the bounding box pipeline in
 *    place.
 *
 *
 * Long Description:
 *    The iterator starts in the waiting state,
 *    when a valid triangle bounding box
 *    appears at the input. It will enter the
 *    testing state the next cycle with a
 *    sample equivelant to the lower left
 *    cooridinate of the bounding box.
 *
 *    While in the testing state, the next sample
 *    for each cycle should be one sample interval
 *    to the right, except when the current sample
 *    is at the right edge.  If the current sample
 *    is at the right edge, the next sample should
 *    be one row up.  Additionally, if the current
 *    sample is on the top row and the right edge,
 *    next cycles sample should be invalid and
 *    equivelant to the lower left vertice and
 *    next cycles state should be waiting.
 *
 *
 *   Author: John Brunhaver
 *   Created:      Thu 07/23/09
 *   Last Updated: Tue 10/01/10
 *
 *   Copyright 2009 <jbrunhaver@gmail.com>
 *
 */

/* ***************************************************************************
 * Change bar:
 * -----------
 * Date           Author    Description
 * Sep 19, 2012   jingpu    ported from John's original code to Genesis
 *
 * ***************************************************************************/

/* A Note on Signal Names:
 *
 * Most signals have a suffix of the form _RxxN
 * where R indicates that it is a Raster Block signal
 * xx indicates the clock slice that it belongs to
 * and N indicates the type of signal that it is.
 * H indicates logic high, L indicates logic low,
 * U indicates unsigned fixed point, and S indicates
 * signed fixed point.
 *
 * For all the signed fixed point signals (logic signed [`$sig_fig`-1:0]),
 * their highest `$sig_fig-$radix` bits, namely [`$sig_fig-1`:`$radix`]
 * represent the integer part of the fixed point number,
 * while the lowest `$radix` bits, namely [`$radix-1`:0]
 * represent the fractional part of the fixed point number.
 *
 *
 *
 * For signal subSample_RnnnnU (logic [3:0])
 * 1000 for  1x MSAA eq to 1 sample per pixel
 * 0100 for  4x MSAA eq to 4 samples per pixel,
 *              a sample is half a pixel on a side
 * 0010 for 16x MSAA eq to 16 sample per pixel,
 *              a sample is a quarter pixel on a side.
 * 0001 for 64x MSAA eq to 64 samples per pixel,
 *              a sample is an eighth of a pixel on a side.
 *
 */

module test_iterator
#(
    parameter SIGFIG = 24, // Bits in color and position.
    parameter RADIX = 10, // Fraction bits in color and position
    parameter VERTS = 3, // Maximum Vertices in triangle
    parameter AXIS = 3, // Number of axis foreach vertex 3 is (x,y,z).
    parameter COLORS = 3, // Number of color channels
    parameter PIPE_DEPTH = 1, // How many pipe stages are in this block
    parameter MOD_FSM = 0 // Use Modified FSM to eliminate a wait state
)
(
    //Input Signals
    input logic signed [SIGFIG-1:0]     tri_R13S[VERTS-1:0][AXIS-1:0], //triangle to Iterate Over
    input logic unsigned [SIGFIG-1:0]   color_R13U[COLORS-1:0] , //Color of triangle
    input logic signed [SIGFIG-1:0]     box_R13S[1:0][1:0], //Box to iterate for subsamples
    input logic                             validTri_R13H, //triangle is valid

    //Control Signals
    input logic [3:0]   subSample_RnnnnU , //Subsample width
    output logic        halt_RnnnnL , //Halt -> hold current microtriangle
    //Note that this block generates
    //Global Signals
    input logic clk, // Clock
    input logic rst, // Reset

    //Outputs
    output logic signed [SIGFIG-1:0]    tri_R14S[VERTS-1:0][AXIS-1:0], //triangle to Sample Test
    output logic unsigned [SIGFIG-1:0]  color_R14U[COLORS-1:0] , //Color of triangle
    output logic signed [SIGFIG-1:0]    sample_R14S[1:0], //Sample Location to Be Tested
    output logic                            validSamp_R14H //Sample and triangle are Valid
);

    // This module implement a Moore machine to iterarte sample points in bbox
    // Recall: a Moore machine is an FSM whose output values are determined
    // solely by its current state.
    // A simple way to build a Moore machine is to make states for every output
    // and the values of the current states are the outputs themselves

    // Now we create the signals for the next states of each outputs and
    // then instantiate registers for storing these states
    logic signed [SIGFIG-1:0]       next_tri_R14S[VERTS-1:0][AXIS-1:0];
    logic unsigned  [SIGFIG-1:0]    next_color_R14U[COLORS-1:0] ;
    logic signed [SIGFIG-1:0]       next_sample_R14S[1:0];
    logic                               next_validSamp_R14H;
    logic                               next_halt_RnnnnL;

    // Instantiate registers for storing these states
    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(VERTS),
        .ARRAY_SIZE2(AXIS),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d301
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (next_tri_R14S  ),
        .out    (tri_R14S       )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(COLORS),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d302
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (next_color_R14U),
        .out    (color_R14U     )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(2),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d303
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (1'b1               ),
        .in     (next_sample_R14S   ),
        .out    (sample_R14S        )
    );

    dff #(
        .WIDTH(2),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0) // No retime
    )
    d304
    (
        .clk    (clk                                    ),
        .reset  (rst                                    ),
        .en     (1'b1                                   ),
        .in     ({next_validSamp_R14H, next_halt_RnnnnL}),
        .out    ({validSamp_R14H, halt_RnnnnL}          )
    );
    // Instantiate registers for storing these states

    typedef enum logic {
                            WAIT_STATE,
                            TEST_STATE
                        } state_t;
generate
if(MOD_FSM == 0) begin // Using baseline FSM
    //////
    //////  RTL code for original FSM Goes Here
    //////

    // To build this FSM we want to have two more state: one is the working
    // status of this FSM, and the other is the current bounding box where
    // we iterate sample points

    // define two more states, box_R14S and state_R14H
    logic signed [SIGFIG-1:0]   box_R14S[1:0][1:0];         // the state for current bounding box
    logic signed [SIGFIG-1:0]   next_box_R14S[1:0][1:0];

    state_t                     state_R14H;     //State Designation (Waiting or Testing)
    state_t                     next_state_R14H;        //Next Cycles State

    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(2),
        .ARRAY_SIZE2(2),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d305
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (next_box_R14S  ),
        .out    (box_R14S       )
    );

    always_ff @(posedge clk, posedge rst) begin
        if(rst) begin
            state_R14H <= WAIT_STATE;
        end
        else begin
            state_R14H <= next_state_R14H;
        end
    end

    // define some helper signals
    logic signed [SIGFIG-1:0]   next_up_samp_R14S[1:0]; //If jump up, next sample
    logic signed [SIGFIG-1:0]   next_rt_samp_R14S[1:0]; //If jump right, next sample
    logic                       at_right_edg_R14H;      //Current sample at right edge of bbox?
    logic                       at_top_edg_R14H;        //Current sample at top edge of bbox?
    logic                       at_end_box_R14H;        //Current sample at end of bbox?

    logic                       is_back_face_R14H; // Current triangle back face?

    //////
    ////// First calculate the values of the helper signals using CURRENT STATES
    //////

    // check the comments 'A Note on Signal Names'
    // at the begining of the module for the help on
    // understanding the signals here
/*
    logic signed [SIGFIG-1:0] x0;
    logic signed [SIGFIG-1:0] x1;
    logic signed [SIGFIG-12:0] x0_1;
    logic signed [SIGFIG-12:0] x1_1;

    logic signed [SIGFIG-1:0] y0;
    logic signed [SIGFIG-1:0] y1;
    logic signed [SIGFIG-12:0] y0_1;
    logic signed [SIGFIG-12:0] y1_1;

    // check if triangle is backfacing
    always_comb begin
        x0 = tri_R13S[1][0] - tri_R13S[0][0];
        x1 = tri_R13S[2][0] - tri_R13S[1][0];
        x0_1 = x0[12:0];
        x1_1 = x1[12:0];

        y0 = tri_R13S[2][1] - tri_R13S[1][1];
        y1 = tri_R13S[1][1] - tri_R13S[0][1];
        y0_1 = y0[12:0];
        y1_1 = y1[12:0];

        //$display("tri_R13S[1][0] - tri_R13S[0][0] %b", x0);
        //$display("tri_R13S[1][0] - tri_R13S[0][0] %b", tri_R13S[1][0][22:0] - tri_R13S[0][0][22:0]);
        //$display("tri_R13S[2][1] - tri_R13S[1][1] %b", y0);
        //$display("tri_R13S[2][1] - tri_R13S[1][1] %b", tri_R13S[2][1][22:0] - tri_R13S[1][1][22:0]);
        //$display("x0 * y0 %b", x0*y0);
        //$display("x0 * y0 %b", x0[22:0]*y0[22:0]);

        if ((x0_1)*(y0_1) - (x1_1)*(y1_1) > 0) is_back_face_R14H = 1'b1;
        else is_back_face_R14H = 1'b0;
    end
*/

    always_comb begin
        // START CODE HERE
        unique case (subSample_RnnnnU)
            4'b1000 : begin
                next_up_samp_R14S[0] = box_R14S[0][0];
                next_up_samp_R14S[1][SIGFIG-1:RADIX] = sample_R14S[1][SIGFIG-1:RADIX] + 1'b1;
                next_up_samp_R14S[1][RADIX-1:0] = sample_R14S[1][RADIX-1:0];

                next_rt_samp_R14S[0][SIGFIG-1:RADIX] = sample_R14S[0][SIGFIG-1:RADIX] + 1'b1;
                next_rt_samp_R14S[0][RADIX-1:0] = sample_R14S[0][RADIX-1:0];
                next_rt_samp_R14S[1] = sample_R14S[1];
            end
           4'b0100 : begin
                next_up_samp_R14S[0] = box_R14S[0][0];
                next_up_samp_R14S[1][SIGFIG-1:RADIX-1] = sample_R14S[1][SIGFIG-1:RADIX-1] + 1'b1;
                next_up_samp_R14S[1][RADIX-2:0] = sample_R14S[1][RADIX-2:0];

                next_rt_samp_R14S[0][SIGFIG-1:RADIX-1] = sample_R14S[0][SIGFIG-1:RADIX-1] + 1'b1;
                next_rt_samp_R14S[0][RADIX-2:0] = sample_R14S[0][RADIX-2:0];
                next_rt_samp_R14S[1] = sample_R14S[1];
            end
            4'b0010 : begin
                next_up_samp_R14S[0] = box_R14S[0][0];
                next_up_samp_R14S[1][SIGFIG-1:RADIX-2] = sample_R14S[1][SIGFIG-1:RADIX-2] + 1'b1;
                next_up_samp_R14S[1][RADIX-3:0] = sample_R14S[1][RADIX-3:0];

                next_rt_samp_R14S[0][SIGFIG-1:RADIX-2] = sample_R14S[0][SIGFIG-1:RADIX-2] + 1'b1;
                next_rt_samp_R14S[0][RADIX-3:0] = sample_R14S[0][RADIX-3:0];
                next_rt_samp_R14S[1] = sample_R14S[1];
            end
            4'b0001 : begin
                next_up_samp_R14S[0] = box_R14S[0][0];
                next_up_samp_R14S[1][SIGFIG-1:RADIX-3] = sample_R14S[1][SIGFIG-1:RADIX-3] + 1'b1;
                next_up_samp_R14S[1][RADIX-4:0] = sample_R14S[1][RADIX-4:0];


                next_rt_samp_R14S[0][SIGFIG-1:RADIX-3] = sample_R14S[0][SIGFIG-1:RADIX-3] + 1'b1;
                next_rt_samp_R14S[0][RADIX-4:0] = sample_R14S[0][RADIX-4:0];
                next_rt_samp_R14S[1] = sample_R14S[1];
            end
        endcase
        if (sample_R14S[0] == box_R14S[1][0]) at_right_edg_R14H = 1'b1;
        else at_right_edg_R14H = 1'b0;

        if (sample_R14S[1] == box_R14S[1][1]) at_top_edg_R14H = 1'b1;
        else at_top_edg_R14H = 1'b0;

        if (sample_R14S == box_R14S[1]) at_end_box_R14H = 1'b1;
        else at_end_box_R14H = 1'b0;


        // END CODE HERE
    end

    //////
    ////// Then complete the following combinational logic defining the
    ////// next states
    //////

    ////// COMPLETE THE FOLLOW ALWAYS_COMB BLOCK

    // Combinational logic for state transitions
    always_comb begin
        // START CODE HERE
        // Try using a case statement on state_R14H
        //$display(state_R14H);
        case (state_R14H)
            WAIT_STATE : begin
                // In waiting state until valid bbox appears at input
                    next_tri_R14S = tri_R13S;
                    next_color_R14U = color_R13U;
                    next_sample_R14S = box_R13S[0];
                    next_validSamp_R14H = (validTri_R13H ? 1'b1 : 1'b0);
                    next_halt_RnnnnL = (validTri_R13H ? 1'b0 : 1'b1);
                    next_box_R14S = box_R13S;
                    next_state_R14H = ((validTri_R13H)? TEST_STATE : WAIT_STATE);
            end
            TEST_STATE : begin
                //$display("beginning of teststate");
                next_tri_R14S = tri_R14S;
                next_box_R14S = box_R14S;
                next_color_R14U = color_R14U;

                if (!at_end_box_R14H && !at_right_edg_R14H) begin
                    next_sample_R14S = next_rt_samp_R14S;     
                    next_validSamp_R14H = 1'b1;
                    next_halt_RnnnnL = 1'b0;
                    next_state_R14H = TEST_STATE;
                end
                else if (!at_end_box_R14H && at_right_edg_R14H) begin
                    next_sample_R14S = next_up_samp_R14S;     
                    next_validSamp_R14H = 1'b1;
                    next_halt_RnnnnL = 1'b0;
                    next_state_R14H = TEST_STATE;
                end
                else begin
                    next_sample_R14S = box_R14S[0];     
                    next_validSamp_R14H = 1'b0;
                    next_halt_RnnnnL = 1'b1;
                    next_state_R14H = WAIT_STATE;
                end
            end
        endcase
        // END CODE HERE
    end // always_comb

    //Assertions for testing FSM logic

    // Write assertions to verify your FSM transition sequence
    // Can you verify that:
    // 1) A validTri_R14H signal causes a transition from WAIT state to TEST state
    // 2) An end_box_R14H signal causes a transition from TEST state to WAIT state
    // 3) What are you missing?

    //Your assertions goes here
    // START CODE HERE
    assert property(@(posedge clk) state_R14H==WAIT_STATE && validTri_R13H==1'b1 |-> next_state_R14H==TEST_STATE);
    assert property(@(posedge clk) state_R14H==TEST_STATE && at_end_box_R14H==1'b1 |-> next_state_R14H==WAIT_STATE);
    // END CODE HERE
    // Assertion ends
    //////
    //////  RTL code for original FSM Finishes
    //////

    //Some Error Checking Assertions

    //Define a Less Than Property
    //
    //  a should be less than b
    property rb_lt( rst, a , b , c );
        @(posedge clk) rst | ((a<=b) | !c);
    endproperty

    //Check that Proposed Sample is in BBox
    // START CODE HERE
    assert property( rb_lt( rst, next_sample_R14S[0], next_box_R14S[1][0], next_validSamp_R14H ));
    assert property( rb_lt( rst, next_sample_R14S[1], next_box_R14S[1][1], next_validSamp_R14H ));

    assert property( rb_lt( rst, next_box_R14S[0][0], next_sample_R14S[0], validTri_R13H ));
    assert property( rb_lt( rst, next_box_R14S[0][1], next_sample_R14S[1], validTri_R13H ));

    // END CODE HERE
    //Check that Proposed Sample is in BBox

    //Error Checking Assertions
end 
else begin // Use modified FSM

    //////
    //////  RTL code for modified FSM Goes Here
    //////

    ////// PLACE YOUR CODE HERE

    //////
    //////  RTL code for modified FSM Finishes
    //////

end
endgenerate

endmodule



