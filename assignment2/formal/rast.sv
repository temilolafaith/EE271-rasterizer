/*
Wrapper for rast to constrain signals
*/
module rast_wrapper
(
    input logic signed [24-1:0]     tri_R10S[3-1:0][3-1:0], // Poly Position
    input logic unsigned [24-1:0]   color_R10U[3-1:0],
    input logic [1:0]               subSample,

    //Global Signals
    input logic clk, // Clock
    input logic rst, // Reset

    //Control Signals
    output logic halt_RnnnnL,

    //Outout Signals
    output logic signed [24-1:0]    hit_R18S[3-1:0], // Output: Sample Location Tested
    output logic unsigned [24-1:0]  color_R18U[3-1:0], // Input: 4 Sets X,Y Fixed Point Values
    output logic                    hit_valid_R18H  // Output: Does Sample lie in uPoly
);

    logic signed [24-1:0]    screen_RnnnnS[1:0]; // Screen Dimensions
    logic                    validTri_R10H;  // Valid Data for Operation
    logic                    cmp_validR10H [2:0] [1:0] [1:0];
    logic                    validTri_R10H_input[1:0][1:0];
    logic [1:0]              subSample_RnnU;
    logic [3:0]              subSample_RnnnnU;

    assign screen_RnnnnS[0] = {1'b1,19'd0};
    assign screen_RnnnnS[1] = {1'b1,19'd0};

    //Constrain subSample to change only at rst
    always @(posedge clk)
    begin
        if(rst) subSample_RnnU <= subSample;
    end

    always_comb begin
        //Convert subSample to one hot
        unique case (subSample_RnnU) // synopsys full_case // synopsys full_case
            2'b00 : subSample_RnnnnU = 4'b0001;
            2'b01 : subSample_RnnnnU = 4'b0010;
            2'b10 : subSample_RnnnnU = 4'b0100;
            2'b11 : subSample_RnnnnU = 4'b1000;
        endcase // case (subSample)

        // Generate validTri_R10H if all coordinates are in screen limits
        cmp_validR10H [2][0][0] = (tri_R10S[2][0] >= 0);
        cmp_validR10H [2][1][0] = (tri_R10S[2][0] <= screen_RnnnnS[0]);
        cmp_validR10H [1][0][0] = (tri_R10S[1][0] >= 0);
        cmp_validR10H [1][1][0] = (tri_R10S[1][0] <= screen_RnnnnS[0]);
        cmp_validR10H [0][0][0] = (tri_R10S[0][0] >= 0);
        cmp_validR10H [0][1][0] = (tri_R10S[0][0] <= screen_RnnnnS[0]);
        cmp_validR10H [2][0][1] = (tri_R10S[2][1] >= 0);
        cmp_validR10H [2][1][1] = (tri_R10S[2][1] >= screen_RnnnnS[1]);
        cmp_validR10H [1][0][1] = (tri_R10S[1][1] >= 0);
        cmp_validR10H [1][1][1] = (tri_R10S[1][1] >= screen_RnnnnS[1]);
        cmp_validR10H [0][0][1] = (tri_R10S[0][1] >= 0);
        cmp_validR10H [0][1][1] = (tri_R10S[0][1] <= screen_RnnnnS[1]);

        validTri_R10H_input[0][0] = (cmp_validR10H [2][0][0] & (cmp_validR10H [1][0][0] & cmp_validR10H [0][0][0]));
        validTri_R10H_input[0][1] = (cmp_validR10H [2][0][1] & (cmp_validR10H [1][0][1] & cmp_validR10H [0][0][1]));
        validTri_R10H_input[1][0] = (cmp_validR10H [2][1][0] & (cmp_validR10H [1][1][0] & cmp_validR10H [0][1][0]));
        validTri_R10H_input[1][1] = (cmp_validR10H [2][1][1] & (cmp_validR10H [1][1][1] & cmp_validR10H [0][1][1]));

        validTri_R10H = validTri_R10H_input[0][0] & (validTri_R10H_input[0][1] & (validTri_R10H_input[1][0] & validTri_R10H_input[1][1]));
    end // always_comb

    rast dut
    (
        .tri_R10S           (tri_R10S           ), // Input: 4 Sets X,Y Fixed Point Values
        .color_R10U         (color_R10U         ), // Input: 4 Sets X,Y Fixed Point Values
        .validTri_R10H      (validTri_R10H      ), // Input: Valid Data for Operation

        .screen_RnnnnS      (screen_RnnnnS      ), // Input: Screen Dimensions
        .subSample_RnnnnU   (subSample_RnnnnU   ), // Input: SubSample_Interval

        .clk                (clk                ), // Input: Clock
        .rst                (rst                ), // Input: Reset

        .halt_RnnnnL        (halt_RnnnnL        ),

        .hit_R18S           (hit_R18S           ), // Output: Sample Location Tested
        .color_R18U         (color_R18U         ), // Input: 4 Sets X,Y Fixed Point Values
        .hit_valid_R18H     (hit_valid_R18H     )  // Output: Does Sample lie in uPoly
    );
endmodule
package rast_params;

    localparam SIGFIG = 24; // Bits in color and position.
    localparam RADIX = 10; // Fraction bits in color and position
    localparam VERTS = 3; // Maximum Vertices in micropolygon
    localparam AXIS = 3; // Number of axis foreach vertex 3 is (x,y,z).
    localparam COLORS = 3; // Number of color channels
    localparam PIPES_BOX = 3; // Number of Pipe Stages in bbox module //3
    localparam PIPES_ITER = 1; // Number of Pipe Stages in iter module //1
    localparam PIPES_HASH = 2; // Number of pipe stages in hash module //2
    localparam PIPES_SAMP = 2; // Number of Pipe Stages in sample module //2

endpackage
/*
 * Bounding Box Module
 *
 * Inputs:
 *   3 x,y,z vertices corresponding to tri
 *   1 valid bit, indicating triangle is valid data
 *
 *  Config Inputs:
 *   2 x,y vertices indicating screen dimensions
 *   1 integer representing square root of SS (16MSAA->4)
 *      we will assume config values are held in some
 *      register and are valid given a valid triangle
 *
 *  Control Input:
 *   1 halt signal indicating that no work should be done
 *
 * Outputs:
 *   2 vertices describing a clamped bounding box
 *   1 Valid signal indicating that bounding
 *           box and triangle value is valid
 *   3 x,y vertices corresponding to tri
 *
 * Global Signals:
 *   clk, rst
 *
 * Function:
 *   Determine a bounding box for the triangle
 *   represented by the vertices.
 *
 *   Clamp the Bounding Box to the subsample pixel
 *   space
 *
 *   Clip the Bounding Box to Screen Space
 *
 *   Halt operating but retain values if next stage is busy
 *
 *
 * Long Description:
 *   This bounding box block accepts a triangle described with three
 *   vertices and determines a set of sample points to test against
 *   the triangle.  These sample points correspond to the
 *   either the pixels in the final image or the pixel fragments
 *   that compose the pixel if multisample anti-aliasing (MSAA)
 *   is enabled.
 *
 *   The inputs to the box are clocked with a bank of dflops.
 *
 *   After the data is clocked, a bounding box is determined
 *   for the triangle. A bounding box can be determined
 *   through calculating the maxima and minima for x and y to
 *   generate a lower left vertice and upper right
 *   vertice.  This data is then clocked.
 *
 *   The bounding box next needs to be clamped to the fragment grid.
 *   This can be accomplished through rounding the bounding box values
 *   to the fragment grid.  Additionally, any sample points that exist
 *   outside of screen space should be rejected.  So the bounding box
 *   can be clipped to the visible screen space.  This clipping is done
 *   using the screen signal.
 *
 *   The Halt signal is utilized to hold the current triangle bounding box.
 *   This is because one bounding box operation could correspond to
 *   multiple sample test operations later in the pipe.  As these samples
 *   can take a number of cycles to complete, the data held in the bounding
 *   box stage needs to be preserved.  The halt signal is also required for
 *   when the write device is full/busy.
 *
 *   The valid signal is utilized to indicate whether or not a triangle
 *   is actual data.  This can be useful if the device being read from,
 *   has no more triangles.
 *
 *
 *
 *   Author: John Brunhaver
 *   Created:      Thu 07/23/09
 *   Last Updated: Fri 09/30/10
 *
 *   Copyright 2009 <jbrunhaver@gmail.com>
 */


/* A Note on Signal Names:
 *
 * Most signals have a suffix of the form _RxxxxN
 * where R indicates that it is a Raster Block signal
 * xxxx indicates the clock slice that it belongs to
 * N indicates the type of signal that it is.
 *    H indicates logic high,
 *    L indicates logic low,
 *    U indicates unsigned fixed point,
 *    S indicates signed fixed point.
 *
 * For all the signed fixed point signals (logic signed [SIGFIG-1:0]),
 * their highest `$sig_fig-$radix` bits, namely [`$sig_fig-1`:RADIX]
 * represent the integer part of the fixed point number,
 * while the lowest RADIX bits, namely [`$radix-1`:0]
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

module bbox
#(
    parameter SIGFIG        = 24, // Bits in color and position.
    parameter RADIX         = 10, // Fraction bits in color and position
    parameter VERTS         = 3, // Maximum Vertices in triangle
    parameter AXIS          = 3, // Number of axis foreach vertex 3 is (x,y,z).
    parameter COLORS        = 3, // Number of color channels
    parameter PIPE_DEPTH    = 3 // How many pipe stages are in this block
)
(
    //Input Signals
    input logic signed [SIGFIG-1:0]     tri_R10S[VERTS-1:0][AXIS-1:0] , // Sets X,Y Fixed Point Values
    input logic unsigned [SIGFIG-1:0]   color_R10U[COLORS-1:0] , // Color of Tri
    input logic                             validTri_R10H , // Valid Data for Operation

    //Control Signals
    input logic                         halt_RnnnnL , // Indicates No Work Should Be Done
    input logic signed [SIGFIG-1:0] screen_RnnnnS[1:0] , // Screen Dimensions
    input logic [3:0]                   subSample_RnnnnU , // SubSample_Interval

    //Global Signals
    input logic clk, // Clock
    input logic rst, // Reset

    //Outout Signals
    output logic signed [SIGFIG-1:0]    tri_R13S[VERTS-1:0][AXIS-1:0], // 4 Sets X,Y Fixed Point Values
    output logic unsigned [SIGFIG-1:0]  color_R13U[COLORS-1:0] , // Color of Tri
    output logic signed [SIGFIG-1:0]    box_R13S[1:0][1:0], // 2 Sets X,Y Fixed Point Values
    output logic                            validTri_R13H                  // Valid Data for Operation
);


    //Signals In Clocking Order

    //Begin R10 Signals

    // Step 1 Result: LL and UR X, Y Fixed Point Values determined by calculating min/max vertices
    // box_R10S[0][0]: LL X
    // box_R10S[0][1]: LL Y
    // box_R10S[1][0]: UR X
    // box_R10S[1][1]: UR Y
    logic signed [SIGFIG-1:0]   box_R10S[1:0][1:0];
    // Step 2 Result: LL and UR Rounded Down to SubSample Interval
    logic signed [SIGFIG-1:0]   rounded_box_R10S[1:0][1:0];
    // Step 3 Result: LL and UR X, Y Fixed Point Values after Clipping
    logic signed [SIGFIG-1:0]   out_box_R10S[1:0][1:0];      // bounds for output
    // Step 3 Result: valid if validTri_R10H && BBox within screen
    logic                           outvalid_R10H;               // output is valid

    //End R10 Signals

    // Begin output for retiming registers
    logic signed [SIGFIG-1:0]   tri_R13S_retime[VERTS-1:0][AXIS-1:0]; // 4 Sets X,Y Fixed Point Values
    logic unsigned [SIGFIG-1:0] color_R13U_retime[COLORS-1:0];        // Color of Tri
    logic signed [SIGFIG-1:0]   box_R13S_retime[1:0][1:0];             // 2 Sets X,Y Fixed Point Values
    logic                           validTri_R13H_retime ;                 // Valid Data for Operation
    // End output for retiming registers

    // ********** Step 0:  Backface Culling**********
    logic backface; 
    //Backfacing if (x1-x0)(y2-y1) > (x2-x1)(y1-y0)
    assign backface = (tri_R10S[1][0] - tri_R10S[0][0])*(tri_R10S[2][1]-tri_R10S[1][1]) > (tri_R10S[2][0] - tri_R10S[1][0])*(tri_R10S[1][1]-tri_R10S[0][1]);
    //ssign backface = (tri_R10S[1][0][SIGFIG-1:0] - tri_R10S[0][0][SIGFIG-1:0])*(tri_R10S[2][1][SIGFIG-1:0]-tri_R10S[1][1][SIGFIG-1:0]) > (tri_R10S[2][0][SIGFIG-1:0] - tri_R10S[1][0][SIGFIG-1:0])*(tri_R10S[1][1][SIGFIG-1:0]-tri_R10S[0][1][SIGFIG-1:0]);


    // ********** Step 1:  Determining a Bounding Box **********
    // Here you need to determine the bounding box by comparing the vertices
    // and assigning box_R10S to be the proper coordinates

    // START CODE HERE

    // This select signal structure may help you in selecting your bbox coordinates
    logic [2:0] bbox_sel_R10H [1:0][1:0];
 
    // The above structure consists of a 3-bit select signal for each coordinate of the 
    // bouding box. The leftmost [1:0] dimensions refer to LL/UR, while the rightmost 
    // [1:0] dimensions refer to X or Y coordinates. Each select signal should be a 3-bit 
    // one-hot signal, where the bit that is high represents which one of the 3 triangle vertices 
    // should be chosen for that bbox coordinate. As an example, if we have: bbox_sel_R10H[0][0] = 3'b001
    // then this indicates that the lower left x-coordinate of your bounding box should be assigned to the 
    // x-coordinate of triangle "vertex a". 
    
    //  DECLARE ANY OTHER SIGNALS YOU NEED
    // logic unsigned [2:0] mask; // mask used to bit and with box_R10S[i][j][RADIX-1:0] (fractional part)
    // Try declaring an always_comb block to assign values to box_R10S
    logic vert_cmp[1:0][2:0]; //compare vertices 

    always_comb begin

        vert_cmp[0][0] = tri_R10S[0][0] < tri_R10S[1][0];
        vert_cmp[0][1] = tri_R10S[0][0] < tri_R10S[2][0];
        vert_cmp[0][2] = tri_R10S[1][0] < tri_R10S[2][0];
        
        vert_cmp[1][0] = tri_R10S[0][1] < tri_R10S[1][1];
        vert_cmp[1][1] = tri_R10S[0][1] < tri_R10S[2][1];
        vert_cmp[1][2] = tri_R10S[1][1] < tri_R10S[2][1];
        //x
        bbox_sel_R10H[0][0][0] =  vert_cmp[0][0] &  vert_cmp[0][1] ;       
        bbox_sel_R10H[0][0][1] = ~vert_cmp[0][0] &  vert_cmp[0][2] ; 
        bbox_sel_R10H[0][0][2] = ~vert_cmp[0][1] & ~vert_cmp[0][2] ; 
        bbox_sel_R10H[1][0][0] = ~vert_cmp[0][0] & ~vert_cmp[0][1] ; 
        bbox_sel_R10H[1][0][1] =  vert_cmp[0][0] & ~vert_cmp[0][2] ; 
        bbox_sel_R10H[1][0][2] =  vert_cmp[0][1] &  vert_cmp[0][2] ; 
        
        // Y
        bbox_sel_R10H[0][1][0] =  vert_cmp[1][0] &  vert_cmp[1][1]  ; 
        bbox_sel_R10H[0][1][1] = ~vert_cmp[1][0] &  vert_cmp[1][2]  ; 
        bbox_sel_R10H[0][1][2] = ~vert_cmp[1][1] & ~vert_cmp[1][2]  ; 
        bbox_sel_R10H[1][1][0] = ~vert_cmp[1][0] & ~vert_cmp[1][1]  ; 
        bbox_sel_R10H[1][1][1] =  vert_cmp[1][0] & ~vert_cmp[1][2]  ; 
        bbox_sel_R10H[1][1][2] =  vert_cmp[1][1] &  vert_cmp[1][2]  ; 

        case(bbox_sel_R10H[0][0])
            3'b001: box_R10S[0][0] = tri_R10S[0][0];
            3'b010: box_R10S[0][0] = tri_R10S[1][0];
            3'b100: box_R10S[0][0] = tri_R10S[2][0];
            default: box_R10S[0][0] = tri_R10S[0][0];
        endcase

        case(bbox_sel_R10H[0][1])
            3'b001: box_R10S[0][1] = tri_R10S[0][1];
            3'b010: box_R10S[0][1] = tri_R10S[1][1];
            3'b100: box_R10S[0][1] = tri_R10S[2][1];
            default: box_R10S[0][1] = tri_R10S[0][1];
        endcase

        case(bbox_sel_R10H[1][0])
            3'b001: box_R10S[1][0] = tri_R10S[0][0];
            3'b010: box_R10S[1][0] = tri_R10S[1][0];
            3'b100: box_R10S[1][0] = tri_R10S[2][0];
            default: box_R10S[1][0] = tri_R10S[0][0];
        endcase

        case(bbox_sel_R10H[1][1])
            3'b001: box_R10S[1][1] = tri_R10S[0][1];
            3'b010: box_R10S[1][1] = tri_R10S[1][1];
            3'b100: box_R10S[1][1] = tri_R10S[2][1];
            default: box_R10S[1][1] = tri_R10S[0][1];
        endcase
    end
    

    // END CODE HERE

    // Assertions to check if box_R10S is assigned properly
    // We want to check the following properties:
    // 1) Each of the coordinates box_R10S are always and uniquely assigned
    // 2) Upper right coordinate is never less than lower left

    // START CODE HERE
    //Assertions to check if all cases are covered and assignments are unique 
    // (already done for you if you use the bbox_sel_R10H select signal as declared)
    assert property(@(posedge clk) $onehot(bbox_sel_R10H[0][0]));
    assert property(@(posedge clk) $onehot(bbox_sel_R10H[0][1]));
    assert property(@(posedge clk) $onehot(bbox_sel_R10H[1][0]));
    assert property(@(posedge clk) $onehot(bbox_sel_R10H[1][1]));

    //Assertions to check UR is never less than LL and that box is valid (assign validTri_R10H???)
    // START CODE HERE
    assert property (@(posedge clk) (box_R10S[0][0] <= box_R10S[1][0]) | !validTri_R10H);
    assert property (@(posedge clk) (box_R10S[0][1] <= box_R10S[1][1]) | !validTri_R10H);

    // END CODE HERE


    // ***************** End of Step 1 *********************


    // ********** Step 2:  Round Values to Subsample Interval **********

    // We will use the floor operation for rounding.
    // To floor a signal, we simply turn all of the bits
    // below a specific RADIX to 0.
    // The complication here is that there are 4 setting.
    // 1x MSAA eq. to 1 sample per pixel
    // 4x MSAA eq to 4 samples per pixel, a sample is
    // half a pixel on a side
    // 16x MSAA eq to 16 sample per pixel, a sample is
    // a quarter pixel on a side.
    // 64x MSAA eq to 64 samples per pixel, a sample is
    // an eighth of a pixel on a side.

    // Note: Cleverly converting the MSAA signal
    //       to a mask would allow you to do this operation
    //       as a bitwise and operation.

    //Round LowerLeft and UpperRight for X and Y
    generate
        for(genvar i = 0; i < 2; i = i + 1) begin
            for(genvar j = 0; j < 2; j = j + 1) begin

                always_comb begin
                    //Integer Portion of LL and UR Remains the Same
                    rounded_box_R10S[i][j][SIGFIG-1:RADIX] = box_R10S[i][j][SIGFIG-1:RADIX];
                    //////// ASSIGN FRACTIONAL PORTION
                    // START CODE HERE
                    unique case (subSample_RnnnnU) // synopsys full_case
                        4'b1000 : begin
                            rounded_box_R10S[i][j][RADIX-1:0] = {RADIX{1'b0}};
                        end
                        4'b0100 : begin
                            rounded_box_R10S[i][j][RADIX-1] = box_R10S[i][j][RADIX-1] ;
                            rounded_box_R10S[i][j][RADIX-2:0] = {RADIX-1{1'b0}};
                        end
                        4'b0010 : begin
                            rounded_box_R10S[i][j][RADIX-1:RADIX-2] = box_R10S[i][j][RADIX-1:RADIX-2];
                            rounded_box_R10S[i][j][RADIX-3:0] = {RADIX-2{1'b0}};
                        end
                        4'b0001 : begin
                            rounded_box_R10S[i][j][RADIX-1:RADIX-3] = box_R10S[i][j][RADIX-1:RADIX-3];
                            rounded_box_R10S[i][j][RADIX-4:0] = {RADIX-3{1'b0}};
                        end
                    endcase
                    // END CODE HERE

                end // always_comb

            end
        end
    endgenerate

    //Assertion to help you debug errors in rounding
    assert property( @(posedge clk) (box_R10S[0][0] - rounded_box_R10S[0][0]) <= {subSample_RnnnnU,7'b0});
    assert property( @(posedge clk) (box_R10S[0][1] - rounded_box_R10S[0][1]) <= {subSample_RnnnnU,7'b0});
    assert property( @(posedge clk) (box_R10S[1][0] - rounded_box_R10S[1][0]) <= {subSample_RnnnnU,7'b0});
    assert property( @(posedge clk) (box_R10S[1][1] - rounded_box_R10S[1][1]) <= {subSample_RnnnnU,7'b0});

    // ***************** End of Step 2 *********************


    // ********** Step 3:  Clipping or Rejection **********

    // Clamp if LL is down/left of screen origin
    // Clamp if UR is up/right of Screen
    // Invalid if BBox is up/right of Screen
    // Invalid if BBox is down/left of Screen
    // outvalid_R10H high if validTri_R10H && BBox is valid
    //logic halt_valid_control;
    
 
    
    always_comb begin

        //////// ASSIGN "out_box_R10S" and "outvalid_R10H"
        // START CODE HERE (use 24'b0???)
        // assign out_box_R10S[1][0] = (rounded_box_R10S[1][0] < screen_RnnnnS[0]) ? rounded_box_R10S[1][0] : screen_RnnnnS[0];
        // assign out_box_R10S[1][1] = (rounded_box_R10S[1][1] < screen_RnnnnS[1]) ? rounded_box_R10S[1][1] : screen_RnnnnS[1];
        // assign out_box_R10S[0][0] = (rounded_box_R10S[0][0] > 1'b0) ? rounded_box_R10S[0][0] : 1'b0;
        // assign out_box_R10S[0][1] = (rounded_box_R10S[0][1] > 1'b0) ? rounded_box_R10S[0][1] : 1'b0;        
        out_box_R10S[0][0] = (box_R10S[0][0] >= 0) ? rounded_box_R10S[0][0] : 0;
        out_box_R10S[0][1] = (box_R10S[0][1] >= 0) ? rounded_box_R10S[0][1] : 0;

        out_box_R10S[1][0] = (box_R10S[1][0] <= screen_RnnnnS[0]) ? rounded_box_R10S[1][0] : screen_RnnnnS[0];
        out_box_R10S[1][1] = (box_R10S[1][1] <= screen_RnnnnS[1]) ? rounded_box_R10S[1][1] : screen_RnnnnS[1];

       if ((out_box_R10S[0][0] >= 0) && (out_box_R10S[0][1] >= 0) && (out_box_R10S[1][0] <= screen_RnnnnS[0]) && (out_box_R10S[1][1] <= screen_RnnnnS[1] && validTri_R10H && !backface))
            outvalid_R10H = 1'b1;
        else
            outvalid_R10H = 1'b0;    

        //change to some sort of and to remove if-else   
        // END CODE HERE    
    end

    //Assertions to check BBox is not totally out of screen
    assert property( @(posedge clk) (out_box_R10S[0][0] >= 0));
    assert property( @(posedge clk) (out_box_R10S[0][1] >= 0));
    assert property( @(posedge clk) (out_box_R10S[1][0] <= screen_RnnnnS[0]));
    assert property( @(posedge clk) (out_box_R10S[1][1] <= screen_RnnnnS[1]));




    //Assertion for checking if outvalid_R10H has been assigned properly
    assert property( @(posedge clk) (outvalid_R10H |-> out_box_R10S[1][0] <= screen_RnnnnS[0]));
    assert property( @(posedge clk) (outvalid_R10H |-> out_box_R10S[1][1] <= screen_RnnnnS[1]));
    //assert property( @(posedge clk) !halt_RnnnnL |-> !outvalid_R10H);

    // ***************** End of Step 3 *********************

    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(VERTS),
        .ARRAY_SIZE2(AXIS),
        .PIPE_DEPTH(PIPE_DEPTH - 1),
        .RETIME_STATUS(1)
    )
    d_bbx_r1
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (halt_RnnnnL        ),
        .in     (tri_R10S          ),
        .out    (tri_R13S_retime   )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(COLORS),
        .PIPE_DEPTH(PIPE_DEPTH - 1),
        .RETIME_STATUS(1)
    )
    d_bbx_r2
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (halt_RnnnnL        ),
        .in     (color_R10U         ),
        .out    (color_R13U_retime  )
    );

    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(2),
        .ARRAY_SIZE2(2),
        .PIPE_DEPTH(PIPE_DEPTH - 1),
        .RETIME_STATUS(1)
    )
    d_bbx_r3
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (halt_RnnnnL    ),
        .in     (out_box_R10S   ),
        .out    (box_R13S_retime)
    );

    dff_retime #(
        .WIDTH(1),
        .PIPE_DEPTH(PIPE_DEPTH - 1),
        .RETIME_STATUS(1) // Retime
    )
    d_bbx_r4
    (
        .clk    (clk                    ),
        .reset  (rst                    ),
        .en     (halt_RnnnnL            ),
        .in     (outvalid_R10H          ),
        .out    (validTri_R13H_retime   )
    );
    //Flop Clamped Box to R13_retime with retiming registers

    //Flop R13_retime to R13 with fixed registers
    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(VERTS),
        .ARRAY_SIZE2(AXIS),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d_bbx_f1
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (halt_RnnnnL        ),
        .in     (tri_R13S_retime    ),
        .out    (tri_R13S           )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(COLORS),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d_bbx_f2
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (halt_RnnnnL        ),
        .in     (color_R13U_retime  ),
        .out    (color_R13U         )
    );

    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(2),
        .ARRAY_SIZE2(2),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d_bbx_f3
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (halt_RnnnnL    ),
        .in     (box_R13S_retime),
        .out    (box_R13S       )
    );

    dff #(
        .WIDTH(1),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0) // No retime
    )
    d_bbx_f4
    (
        .clk    (clk                    ),
        .reset  (rst                    ),
        .en     (halt_RnnnnL            ),
        .in     (validTri_R13H_retime   ),
        .out    (validTri_R13H          )
    );
    //Flop R13_retime to R13 with fixed registers

    //Error Checking Assertions

    //Define a Less Than Property
    //
    //  a should be less than b
    property rb_lt( rst, a, b, c );
        @(posedge clk) rst | ((a<=b) | !c);
    endproperty

    //Check that Lower Left of Bounding Box is less than equal Upper Right
    assert property( rb_lt( rst, box_R13S[0][0], box_R13S[1][0], validTri_R13H ));
    assert property( rb_lt( rst, box_R13S[0][1], box_R13S[1][1], validTri_R13H ));
    //Check that Lower Left of Bounding Box is less than equal Upper Right

    //Error Checking Assertions

endmodule/*
 This is a two dimentional array of DFFs
 */

/* ***************************************************************************
 * Change bar:
 * -----------
 * Date           Author    Description
 * Sep 20, 2012   jingpu    init version
 *
 * ***************************************************************************/

/******************************************************************************
 * PARAMETERIZATION
 * ***************************************************************************/
//; # module parameters
//; my $bitwidth        = parameter(Name=>'BitWidth',
//;                                 Val=>64, Min=>1, Step=>1,
//;                                 Doc=>"Signal bit widths");
//; my $array_size      = parameter(Name=>'ArraySize1',
//;                                 Val=>64, Min=>1, Step=>1,
//;                                 Doc=>"The size of array's first dimention");
//; my $pipe_depth      = parameter(Name=>'PipelineDepth',
//;                                 Val=>1, Min=>0, Step=>1,
//;                                 Doc=>"Pipeline depth");
//; my $retime_Status   = parameter(Name=>'Retime' ,
//;                                 Val=>'NO' , List=>[ 'YES' , 'NO' ] ,
//;                                 Doc=>"Pipeline Is Retimeable" ) ;

module dff2
#(
    parameter WIDTH = 64,
    parameter ARRAY_SIZE = 64,
    parameter PIPE_DEPTH = 1,
    parameter RETIME_STATUS = 0
)
(
    input logic [WIDTH-1:0]  in[ARRAY_SIZE-1:0],
    input logic clk,
    input logic reset,
    input logic en,
    output logic [WIDTH-1:0] out[ARRAY_SIZE-1:0]
);

generate
for(genvar i = 0; i < ARRAY_SIZE; i = i + 1) begin
    if(RETIME_STATUS) begin
        dff_retime #(
            .WIDTH          (WIDTH          ),
            .PIPE_DEPTH     (PIPE_DEPTH     ),
            .RETIME_STATUS  (RETIME_STATUS  )
        )
        dff_pipe
        (
            .clk    (clk        ),
            .reset  (reset      ),
            .en     (en         ),
            .in     (in[i]      ),
            .out    (out[i]     )
        );
    end
    else begin
        dff #(
            .WIDTH          (WIDTH          ),
            .PIPE_DEPTH     (PIPE_DEPTH     ),
            .RETIME_STATUS  (RETIME_STATUS  )
        )
        dff_pipe
        (
            .clk    (clk        ),
            .reset  (reset      ),
            .en     (en         ),
            .in     (in[i]      ),
            .out    (out[i]     )
        );
    end
end
endgenerate
   //; }

endmodule
/*
 This is a three dimentional array of DFFs
 */

/* ***************************************************************************
 * Change bar:
 * -----------
 * Date           Author    Description
 * Sep 20, 2012   jingpu    init version
 *
 * ***************************************************************************/

/******************************************************************************
 * PARAMETERIZATION
 * ***************************************************************************/
//; # module parameters
//; my $bitwidth        = parameter(Name=>'BitWidth',
//;                                 Val=>64, Min=>1, Step=>1,
//;                                 Doc=>"Signal bit widths");
//; my $array_size1     = parameter(Name=>'ArraySize1',
//;                                 Val=>64, Min=>1, Step=>1,
//;                                 Doc=>"The size of array's first dimention");
//; my $array_size2     = parameter(Name=>'ArraySize2',
//;                                 Val=>64, Min=>1, Step=>1,
//;                                 Doc=>"The size of array's second dimention");
//; my $pipe_depth      = parameter(Name=>'PipelineDepth',
//;                                 Val=>1, Min=>0, Step=>1,
//;                                 Doc=>"Pipeline depth");
//; my $retime_Status   = parameter(Name=>'Retime' ,
//;                                 Val=>'NO' , List=>[ 'YES' , 'NO' ] ,
//;                                 Doc=>"Pipeline Is Retimeable" ) ;

module dff3
#(
    parameter WIDTH = 64,
    parameter ARRAY_SIZE1 = 64,
    parameter ARRAY_SIZE2 = 64,
    parameter PIPE_DEPTH = 1,
    parameter RETIME_STATUS = 0
)
(
    input logic [WIDTH-1:0]  in[ARRAY_SIZE1-1:0][ARRAY_SIZE2-1:0],
    input logic clk,
    input logic reset,
    input logic en,
    output logic [WIDTH-1:0] out[ARRAY_SIZE1-1:0][ARRAY_SIZE2-1:0]
);

generate
for(genvar i = 0; i < ARRAY_SIZE1; i = i + 1) begin
    dff2 #(
        .WIDTH(WIDTH),
        .ARRAY_SIZE(ARRAY_SIZE2),
        .PIPE_DEPTH(PIPE_DEPTH),
        .RETIME_STATUS(RETIME_STATUS)
    )
    dff_arr
    (
        .clk    (clk        ),
        .reset  (reset      ),
        .en     (en         ),
        .in     (in[i]      ),
        .out    (out[i]     )
    );
end
endgenerate

endmodule
/*
 This is a Genesis wrapper of DW pipeline regs with en singal
 */

/* ***************************************************************************
 * Change bar:
 * -----------
 * Date           Author    Description
 * Sep 20, 2012   jingpu    init version
 *
 * ***************************************************************************/

module dff_retime
#(
    parameter WIDTH = 64, // Signal bit widths
    parameter PIPE_DEPTH = 1, // Pipeline depth
    parameter RETIME_STATUS = 0 // Pipeline Is Retimeable
)
(
    input logic [WIDTH - 1:0]  in,
    input logic clk,
    input logic reset,
    input logic en,
    output logic [WIDTH - 1:0] out
);

// Compiler pragmas for retiming

   /* synopsys dc_tcl_script_begin
       set_ungroup [current_design] true
       set_flatten true -effort high -phase true -design [current_design]
       set_dont_retime [current_design] false
       set_optimize_registers true -design [current_design]
    */


generate
if(PIPE_DEPTH > 0) begin

    DW_pl_reg
    #(
        .stages     (PIPE_DEPTH + 1 ),
        .in_reg     (0              ),
        .out_reg    (0              ),
        .width      (WIDTH          ),
        .rst_mode   (0              )
    )
    dff_pipe
    (
        .clk        (clk                ),
        .rst_n      (!reset             ),
        .data_in    (in                 ),
        .data_out   (out                ),
        .enable     ({PIPE_DEPTH{en}})
    );

end 
else begin
    assign out = in & (~{WIDTH{reset}});
end
endgenerate

endmodule
/*
 This is a Genesis wrapper of DW pipeline regs with en singal
 */

/* ***************************************************************************
 * Change bar:
 * -----------
 * Date           Author    Description
 * Sep 20, 2012   jingpu    init version
 *
 * ***************************************************************************/

module dff
#(
    parameter WIDTH = 64, // Signal bit widths
    parameter PIPE_DEPTH = 1, // Pipeline depth
    parameter RETIME_STATUS = 0 // Pipeline Is Retimeable
)
(
    input logic [WIDTH - 1:0]  in,
    input logic clk,
    input logic reset,
    input logic en,
    output logic [WIDTH - 1:0] out
);

// Compiler pragmas for retiming
   /* synopsys dc_tcl_script_begin
       set_ungroup [current_design] true
       set_flatten true -effort high -phase true -design [current_design]
       set_dont_retime [current_design] true
       set_optimize_registers false -design [current_design]
    */

generate
if(PIPE_DEPTH > 0) begin

    DW_pl_reg
    #(
        .stages     (PIPE_DEPTH + 1 ),
        .in_reg     (0              ),
        .out_reg    (0              ),
        .width      (WIDTH          ),
        .rst_mode   (0              )
    )
    dff_pipe
    (
        .clk        (clk                ),
        .rst_n      (!reset             ),
        .data_in    (in                 ),
        .data_out   (out                ),
        .enable     ({PIPE_DEPTH{en}})
    );

end 
else begin
    assign out = in & (~{WIDTH{reset}});
end
endgenerate

endmodule
/*
 *  Hashing Function
 *
 *  Inputs:
 *    triangle and Sample Information
 *
 *  Outputs:
 *    Jittered Sample Position and Buffered triangle
 *
 *  Function:
 *    Calc on offset for the sample.  This is used for
 *    stochastic sampling reasons.  Note that this is
 *    a simplified hashing mechanism.  An in depth
 *    discussion of stochastic sampling in rendering
 *    can be found here:
 *    http://doi.acm.org/10.1145/7529.8927
 *
 *
 * Long Description:
 *    The basic idea is to use a tree of xor
 *    functions to generate a displacement
 *    from the sample center.
 *
 *
 *   Author: John Brunhaver
 *   Created:      Thu 10/01/10
 *   Last Updated: Tue 10/15/10
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
 */

module hash_jtree
#(
    parameter SIGFIG = 24,
    parameter RADIX = 10,
    parameter VERTS = 3,
    parameter AXIS = 3,
    parameter COLORS = 3,
    parameter PIPE_DEPTH = 3
)
(
    //Input Signals
    input logic signed    [SIGFIG-1:0]  tri_R14S[VERTS-1:0][AXIS-1:0],  //triangle to Sample Test
    input logic unsigned  [SIGFIG-1:0]  color_R14U[COLORS-1:0],         //Color of Tri
    input logic signed    [SIGFIG-1:0]  sample_R14S[1:0],                //Sample Location to Be Tested
    input logic                         validSamp_R14H,                  //Sample and triangle are Valid

    //Global Signals
    input logic clk, // Clock
    input logic rst, // Reset

    //Control Signals
    input logic [3:0] subSample_RnnnnU ,   //Subsample width

    //Outputs
    output logic signed   [SIGFIG-1:0]  tri_R16S[VERTS-1:0][AXIS-1:0], // triangle to Iterate Over
    output logic unsigned [SIGFIG-1:0]  color_R16U[COLORS-1:0],        // Color of Tri
    output logic signed   [SIGFIG-1:0]  sample_R16S[1:0],              // Sample Location
    output logic                        validSamp_R16H                 // A valid sample location
);

    localparam HASH_IN_WIDTH = (SIGFIG - 4) * 2;
    localparam HASH_OUT_WIDTH = RADIX - 2;

    // output for retiming registers
    logic signed [SIGFIG-1:0]   tri_R16S_retime[VERTS-1:0][AXIS-1:0]; // triangle to Iterate Over
    logic unsigned [SIGFIG-1:0] color_R16U_retime[COLORS-1:0];      // Color of Tri
    logic signed [SIGFIG-1:0]   sample_R16S_retime[1:0];    // Sample Location
    logic                       validSamp_R16H_retime;      // A valid sample location
    // output for retiming registers

    logic [HASH_OUT_WIDTH-1:0]  hash_mask_R14H ;
    logic [HASH_OUT_WIDTH-1:0]  jitt_val_R14H[1:0] ;
    logic [SIGFIG-1:0]          sample_jitted_R14S[1:0] ;

    always_comb begin
        assert( $onehot(subSample_RnnnnU) ) ;
        unique case ( 1'b1 ) // synopsys full_case
            (subSample_RnnnnU[3]): hash_mask_R14H = 8'b11111111 ; //MSAA = 1
            (subSample_RnnnnU[2]): hash_mask_R14H = 8'b01111111 ; //MSAA = 4
            (subSample_RnnnnU[1]): hash_mask_R14H = 8'b00111111 ; //MSAA = 16
            (subSample_RnnnnU[0]): hash_mask_R14H = 8'b00011111 ; //MSAA = 64
        endcase // case ( 1'b1 )
    end

    tree_hash #(
        .IN_WIDTH(HASH_IN_WIDTH),
        .OUT_WIDTH(HASH_OUT_WIDTH)
    )
    xjit_hash
    (
        .in_RnnH    ({sample_R14S[1][SIGFIG-1:4],
                      sample_R14S[0][SIGFIG-1:4]}   ),
        .mask_RnnH  (hash_mask_R14H                 ),
        .out_RnnH   (jitt_val_R14H[0]               )
    );

    tree_hash #(
        .IN_WIDTH(HASH_IN_WIDTH),
        .OUT_WIDTH(HASH_OUT_WIDTH)
    )
    yjit_hash
    (
        .in_RnnH    ({sample_R14S[0][SIGFIG-1:4],
                      sample_R14S[1][SIGFIG-1:4]}   ),
        .mask_RnnH  (hash_mask_R14H                     ),
        .out_RnnH   (jitt_val_R14H[1]                   )
    );

    //Jitter the sample coordinates
    assign sample_jitted_R14S[0] =   { sample_R14S[0][SIGFIG-1:0] }
                                    | { {(SIGFIG - RADIX){1'b0}},                 //23:10 = 14 bits
                                        jitt_val_R14H[0][HASH_OUT_WIDTH-1:0], //7:0 = 8 bits
                                        {(RADIX - HASH_OUT_WIDTH){1'b0}} };     //1:0 = 2 bits  ==> 24 bits total

    //Jitter the sample coordinates
    assign sample_jitted_R14S[1] =   { sample_R14S[1][SIGFIG-1:0] }
                                    | { {(SIGFIG - RADIX){1'b0}},                 //23:10 = 14 bits
                                        jitt_val_R14H[1][HASH_OUT_WIDTH-1:0], //7:0 = 8 bits
                                        {(RADIX - HASH_OUT_WIDTH){1'b0}} };     //1:0 = 2 bits  ==> 24 bits total

    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(VERTS),
        .ARRAY_SIZE2(AXIS),
        .PIPE_DEPTH(PIPE_DEPTH - 1),
        .RETIME_STATUS(1)
    )
    d_hash_r1
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (1'b1               ),
        .in     (tri_R14S           ),
        .out    (tri_R16S_retime    )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(COLORS),
        .PIPE_DEPTH(PIPE_DEPTH - 1),
        .RETIME_STATUS(1)
    )
    d_hash_r2
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (1'b1               ),
        .in     (color_R14U         ),
        .out    (color_R16U_retime  )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(2),
        .PIPE_DEPTH(PIPE_DEPTH - 1),
        .RETIME_STATUS(1)
    )
    d_hash_r3
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (1'b1               ),
        .in     (sample_jitted_R14S ),
        .out    (sample_R16S_retime )
    );

    dff_retime #(
        .WIDTH(1),
        .PIPE_DEPTH(PIPE_DEPTH - 1),
        .RETIME_STATUS(1) // Retime
    )
    d_hash_r4
    (
        .clk    (clk                    ),
        .reset  (rst                    ),
        .en     (1'b1                   ),
        .in     (validSamp_R14H         ),
        .out    (validSamp_R16H_retime  )
    );
    //Flop R14 to R16_retime with retiming registers

    //Flop R16_retime to R16 with fixed registers
    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(VERTS),
        .ARRAY_SIZE2(AXIS),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d_hash_f1
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (1'b1               ),
        .in     (tri_R16S_retime    ),
        .out    (tri_R16S           )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(COLORS),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d_hash_f2
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (1'b1               ),
        .in     (color_R16U_retime  ),
        .out    (color_R16U         )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(2),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d_hash_f3
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (1'b1               ),
        .in     (sample_R16S_retime ),
        .out    (sample_R16S        )
    );

    dff #(
        .WIDTH(1),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0) // No retime
    )
    d_hash_f4
    (
        .clk    (clk                    ),
        .reset  (rst                    ),
        .en     (1'b1                   ),
        .in     (validSamp_R16H_retime  ),
        .out    (validSamp_R16H         )
    );
    //Flop R16_retime to R16 with fixed registers

endmodule
/*
 * Reyes Style Hider:
 *
 *  This module accepts a stream of triangles
 *  and produces a stream of fragments
 *
 *  This module contains three submodules:
 *    -bounding box module which generates the bounding box
 *     for a triangle
 *    -test iterator module which iterates over the bounding
 *    -sample test function which tests to see if the sample
 *     location from the bounding box lay inside the triangle
 *
 *
 *   Author: John Brunhaver
 *   Created:          09/21/09
 *   Last Updated: TUE 10/20/09
 *
 *   Copyright 2009 <jbrunhaver@gmail.com>
 */


/* ***************************************************************************
 * Change bar:
 * -----------
 * Date           Author    Description
 * Sep 19, 2012   jingpu    ported from John's original code to Genesis
 *
 * ***************************************************************************/
//`include "rast_params.sv"
import rast_params::*;

module rast
#(
    parameter SIGFIG = rast_params::SIGFIG, // Bits in color and position.
    parameter RADIX = rast_params::RADIX, // Fraction bits in color and position
    parameter VERTS = rast_params::VERTS, // Maximum Vertices in triangle
    parameter AXIS = rast_params::AXIS, // Number of axis foreach vertex 3 is (x,y,z).
    parameter COLORS = rast_params::COLORS, // Number of color channels
    parameter PIPES_BOX = rast_params::PIPES_BOX, // Number of Pipe Stages in bbox module
    parameter PIPES_ITER = rast_params::PIPES_ITER, // Number of Pipe Stages in iter module
    parameter PIPES_HASH = rast_params::PIPES_HASH, // Number of pipe stages in hash module
    parameter PIPES_SAMP = rast_params::PIPES_SAMP // Number of Pipe Stages in sample module
)
(
    // Input Signals
    input logic signed [SIGFIG-1:0]     tri_R10S[VERTS-1:0][AXIS-1:0], // Tri Position
    input logic unsigned [SIGFIG-1:0]   color_R10U[COLORS-1:0], // Color of Tri
    input logic                             validTri_R10H, // Valid Data for Operation

    // Input Control Signals ( ala CSR )
    input logic signed [SIGFIG-1:0] screen_RnnnnS[1:0], // Screen Dimensions
    input logic [3:0]                   subSample_RnnnnU, // SubSample_Interval

    // Global Signals
    input logic clk, // Clock
    input logic rst, // Reset

    // Output Control Signals
    output logic halt_RnnnnL,

    // Output Signals
    output logic signed [SIGFIG-1:0]    hit_R18S[AXIS-1:0], // Hit Location
    output logic unsigned [SIGFIG-1:0]  color_R18U[COLORS-1:0], // Color of Tri
    output logic                            hit_valid_R18H            // Is this a hit?
);
    `ifdef GENERATE_JSON
    integer bbox_file;
    integer sample_file;
    integer hash_file;
    integer iterator_file;
    integer rast_file;

    initial begin
        bbox_file = $fopen("ComputeBoundingBox_vector.json", "w");
        $fdisplay(bbox_file, "[");

        sample_file = $fopen("SampleTest_vector.json", "w");
        $fdisplay(sample_file, "[");

        hash_file = $fopen("HashJTree_vector.json", "w");
        $fdisplay(hash_file, "[");

        iterator_file = $fopen("Iterator_vector.json", "w");
        $fdisplay(iterator_file, "[");

        rast_file = $fopen("Rasterizer_vector.json", "w");
        $fdisplay(rast_file, "[");
    end

    always @(posedge clk) begin
        $fdisplay(rast_file, "{\"RESET\": \"%b\",", rst);

        $fdisplay(rast_file, "\"valid_in\": \"%b\",", validTri_R10H);

        $fdisplay(rast_file, "\"tri\": [[\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"]]",
        tri_R10S[0][0],
        tri_R10S[0][1],
        tri_R10S[0][2],
        tri_R10S[1][0],
        tri_R10S[1][1],
        tri_R10S[1][2],
        tri_R10S[2][0],
        tri_R10S[2][1],
        tri_R10S[2][2], 
        ",");

        $fdisplay(rast_file, "\"color_in\": [\"%b\", \"%b\", \"%b\"]",
        color_R10U[0],
        color_R10U[1],
        color_R10U[2], 
        ",");

        $fdisplay(rast_file, "\"screen_max\": [\"%b\", \"%b\"],", screen_RnnnnS[0], screen_RnnnnS[1]);
        $fdisplay(rast_file, "\"sample_size\": \"%b\",", subSample_RnnnnU);
        $fdisplay(rast_file, "\"halt\": \"%b\",", halt_RnnnnL);
        $fdisplay(rast_file, "\"valid_hit\": \"%b\",", hit_valid_R18H);

        $fdisplay(rast_file, "\"hit\": [\"%b\", \"%b\", \"%b\"]",
        hit_R18S[0],
        hit_R18S[1],
        hit_R18S[2],
        ",");

        $fdisplay(rast_file, "\"color_out\": [\"%b\", \"%b\", \"%b\"]},",
        color_R18U[0],
        color_R18U[1],
        color_R18U[2]);
     //   ",");

       // $fdisplay(rast_file, "},");
    end
    `endif

    //Intermediate Signals
    logic signed [SIGFIG-1:0]   box_R13S[1:0][1:0];             // 2 Sets X,Y Fixed Point Values
    logic signed [SIGFIG-1:0]   tri_R13S[VERTS-1:0][AXIS-1:0]; // 4 Sets X,Y Fixed Point Values
    logic unsigned [SIGFIG-1:0] color_R13U[COLORS-1:0]  ;       // Color of Tri
    logic                           validTri_R13H;                 // Valid Data for Operation

    logic signed [SIGFIG-1:0]   tri_R14S[VERTS-1:0][AXIS-1:0]; //triangle to Sample Test
    logic unsigned [SIGFIG-1:0] color_R14U[COLORS-1:0] ;         // Color of Tri
    logic signed [SIGFIG-1:0]   sample_R14S[1:0];               //Sample Location to Be Tested
    logic                           validSamp_R14H;                 //Sample and triangle are Valid

    logic signed [SIGFIG-1:0]   tri_R16S[VERTS-1:0][AXIS-1:0]; //triangle to Sample Test
    logic unsigned [SIGFIG-1:0] color_R16U[COLORS-1:0] ;         //Color of Tri
    logic signed [SIGFIG-1:0]   sample_R16S[1:0];               //Sample Location to Be Tested
    logic                           validSamp_R16H;                 //Sample and triangle are Valid

    logic [SIGFIG-1:0]  zero;                     //fudge signal to hold zero as a reset value
    logic [127:0]           big_zero;                 //fudge signal to hold zero as a reset value
    //Intermediate Signals

    assign big_zero = 128'd0;
    assign zero = big_zero[SIGFIG-1:0];

    //TODO: Missing triangle color

    //TODO: Make param pipedepth work

    bbox #(
        .SIGFIG     (SIGFIG     ),
        .RADIX      (RADIX      ),
        .VERTS      (VERTS      ),
        .AXIS       (AXIS       ),
        .COLORS     (COLORS     ),
        .PIPE_DEPTH (PIPES_BOX  )
    )
    bbox
    (
        .tri_R10S           (tri_R10S           ),
        .color_R10U         (color_R10U         ),
        .validTri_R10H      (validTri_R10H      ),

        .halt_RnnnnL        (halt_RnnnnL        ),
        .screen_RnnnnS      (screen_RnnnnS      ),
        .subSample_RnnnnU   (subSample_RnnnnU   ),

        .clk                (clk                ),
        .rst                (rst                ),

        .tri_R13S           (tri_R13S           ),
        .color_R13U         (color_R13U         ),
        .box_R13S           (box_R13S           ),
        .validTri_R13H      (validTri_R13H      )
    );

    // Generating vectors for bbox
    // 'RESET': Reset, 'valid_in': Bits(1), 'tri_in':
    // Array(3,Array(3,Out(SInt(24)))), 'color_in': Array(3,Out(UInt(24))),
    // 'screen_max': Array(2,Out(SInt(24))), 'sample_size': Bits(4), 'halt':
    // Bits(1), 'valid_out': Bits(1), 'tri_out':
    // Array(3,Array(3,In(SInt(24)))), 'color_out': Array(3,In(UInt(24))),
    // 'box': Array(2,Array(2,In(SInt(24)))), 'is_quad_in': Bits(1),
    // 'is_quad_out': Bits(1)}
    `ifdef GENERATE_JSON
    always @(posedge clk) begin
        $fdisplay(bbox_file, "{\"RESET\": \"%b\",", rst);

        $fdisplay(bbox_file, "\"valid_in\": \"%b\",", validTri_R10H);

        $fdisplay(bbox_file, "\"tri_in\": [[\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"]]",
        tri_R10S[0][0],
        tri_R10S[0][1],
        tri_R10S[0][2],
        tri_R10S[1][0],
        tri_R10S[1][1],
        tri_R10S[1][2],
        tri_R10S[2][0],
        tri_R10S[2][1],
        tri_R10S[2][2], 
        ",");

        $fdisplay(bbox_file, "\"color_in\": [\"%b\", \"%b\", \"%b\"]",
        color_R10U[0],
        color_R10U[1],
        color_R10U[2], 
        ",");

        $fdisplay(bbox_file, "\"screen_max\": [\"%b\", \"%b\"],", screen_RnnnnS[0], screen_RnnnnS[1]);

        $fdisplay(bbox_file, "\"sample_size\": \"%b\",", subSample_RnnnnU);

        $fdisplay(bbox_file, "\"halt\": \"%b\",", halt_RnnnnL);

        $fdisplay(bbox_file, "\"valid_out\": \"%b\",", validTri_R13H);

        $fdisplay(bbox_file, "\"tri_out\": [[\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"]]",
        tri_R13S[0][0],
        tri_R13S[0][1],
        tri_R13S[0][2],
        tri_R13S[1][0],
        tri_R13S[1][1],
        tri_R13S[1][2],
        tri_R13S[2][0],
        tri_R13S[2][1],
        tri_R13S[2][2], 
        ",");

        $fdisplay(bbox_file, "\"color_out\": [\"%b\", \"%b\", \"%b\"]",
        color_R13U[0],
        color_R13U[1],
        color_R13U[2],
        ",");

        $fdisplay(bbox_file, "\"box\": [[\"%b\", \"%b\"], [\"%b\", \"%b\"]]",
        box_R13S[0][0],
        box_R13S[0][1],
        box_R13S[1][0],
        box_R13S[1][1]);
        //
        //",");

        $fdisplay(bbox_file, "},");
    end
    `endif 

    test_iterator #(
        .SIGFIG     (SIGFIG     ),
        .RADIX      (RADIX      ),
        .VERTS      (VERTS      ),
        .AXIS       (AXIS       ),
        .COLORS     (COLORS     ),
        .PIPE_DEPTH (PIPES_ITER )
    )
    test_iterator
    (
        .tri_R13S           (tri_R13S           ),
        .color_R13U         (color_R13U         ),
        .box_R13S           (box_R13S           ),
        .validTri_R13H      (validTri_R13H      ),

        .subSample_RnnnnU   (subSample_RnnnnU   ),
        .halt_RnnnnL        (halt_RnnnnL        ),

        .clk                (clk                ),
        .rst                (rst                ),

        .tri_R14S           (tri_R14S           ),
        .color_R14U         (color_R14U         ),
        .sample_R14S        (sample_R14S        ),
        .validSamp_R14H     (validSamp_R14H     )
    );

    `ifdef GENERATE_JSON
    always @(posedge clk) begin
        $fdisplay(iterator_file, "{\"RESET\": \"%b\",", rst);

        $fdisplay(iterator_file, "\"tri_in\": [[\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"]]",
        tri_R13S[0][0],
        tri_R13S[0][1],
        tri_R13S[0][2],
        tri_R13S[1][0],
        tri_R13S[1][1],
        tri_R13S[1][2],
        tri_R13S[2][0],
        tri_R13S[2][1],
        tri_R13S[2][2], 
        ",");

        $fdisplay(iterator_file, "\"color_in\": [\"%b\", \"%b\", \"%b\"]",
        color_R13U[0],
        color_R13U[1],
        color_R13U[2],
        ",");

        $fdisplay(iterator_file, "\"valid_in\": \"%b\",", validTri_R13H);

        $fdisplay(iterator_file, "\"box\": [[\"%b\", \"%b\"], [\"%b\", \"%b\"]]",
        box_R13S[0][0],
        box_R13S[0][1],
        box_R13S[1][0],
        box_R13S[1][1],
        ",");

        $fdisplay(iterator_file, "\"sample_size\": \"%b\",", subSample_RnnnnU);
        $fdisplay(iterator_file, "\"halt\": \"%b\",", halt_RnnnnL);

        $fdisplay(iterator_file, "\"tri_out\": [[\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"]]",
        tri_R14S[0][0],
        tri_R14S[0][1],
        tri_R14S[0][2],
        tri_R14S[1][0],
        tri_R14S[1][1],
        tri_R14S[1][2],
        tri_R14S[2][0],
        tri_R14S[2][1],
        tri_R14S[2][2], 
        ",");

        $fdisplay(iterator_file, "\"color_out\": [\"%b\", \"%b\", \"%b\"]",
        color_R14U[0],
        color_R14U[1],
        color_R14U[2],
        ",");

        $fdisplay(iterator_file, "\"sample\": [\"%b\", \"%b\"]",
        sample_R14S[0],
        sample_R14S[1],
        ",");

        $fdisplay(iterator_file, "\"valid_sample\": \"%b\"},", validSamp_R14H);

        //$fdisplay(iterator_file, ",");
    end 
    `endif

    hash_jtree #(
        .SIGFIG     (SIGFIG     ),
        .RADIX      (RADIX      ),
        .VERTS      (VERTS      ),
        .AXIS       (AXIS       ),
        .COLORS     (COLORS     ),
        .PIPE_DEPTH (PIPES_HASH )
    )
    hash_jtree
    (
        .tri_R14S           (tri_R14S           ),
        .color_R14U         (color_R14U         ),
        .sample_R14S        (sample_R14S        ),
        .validSamp_R14H     (validSamp_R14H     ),

        .subSample_RnnnnU   (subSample_RnnnnU   ),

        .clk                (clk                ),
        .rst                (rst                ),

        .tri_R16S           (tri_R16S           ),
        .color_R16U         (color_R16U         ),
        .sample_R16S        (sample_R16S        ),
        .validSamp_R16H     (validSamp_R16H     )
    );

    `ifdef GENERATE_JSON
    always @(posedge clk) begin
        $fdisplay(hash_file, "{\"RESET\": \"%b\",", rst);

        $fdisplay(hash_file, "\"tri_in\": [[\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"]]",
        tri_R14S[0][0],
        tri_R14S[0][1],
        tri_R14S[0][2],
        tri_R14S[1][0],
        tri_R14S[1][1],
        tri_R14S[1][2],
        tri_R14S[2][0],
        tri_R14S[2][1],
        tri_R14S[2][2], 
        ",");

        $fdisplay(hash_file, "\"color_in\": [\"%b\", \"%b\", \"%b\"]",
        color_R14U[0],
        color_R14U[1],
        color_R14U[2],
        ",");

        $fdisplay(hash_file, "\"sample_in\": [\"%b\", \"%b\"]",
        sample_R14S[0],
        sample_R14S[1],
        ",");

        $fdisplay(hash_file, "\"valid_sample_in\": \"%b\",", validSamp_R14H);

        $fdisplay(hash_file, "\"sample_size\": \"%b\",", subSample_RnnnnU);

        $fdisplay(hash_file, "\"tri_out\": [[\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"], [\"%b\", \"%b\", \"%b\"]]",
        tri_R16S[0][0],
        tri_R16S[0][1],
        tri_R16S[0][2],
        tri_R16S[1][0],
        tri_R16S[1][1],
        tri_R16S[1][2],
        tri_R16S[2][0],
        tri_R16S[2][1],
        tri_R16S[2][2], 
        ",");

        $fdisplay(hash_file, "\"color_out\": [\"%b\", \"%b\", \"%b\"]",
        color_R16U[0],
        color_R16U[1],
        color_R16U[2],
        ",");

        $fdisplay(hash_file, "\"sample_out\": [\"%b\", \"%b\"]",
        sample_R16S[0],
        sample_R16S[1],
        ",");

        $fdisplay(hash_file, "\"valid_sample_out\": \"%b\"},", validSamp_R16H);

       // $fdisplay(hash_file, ",");
    end 
    `endif

    sampletest #(
        .SIGFIG     (SIGFIG     ),
        .RADIX      (RADIX      ),
        .VERTS      (VERTS      ),
        .AXIS       (AXIS       ),
        .COLORS     (COLORS     ),
        .PIPE_DEPTH (PIPES_SAMP )
    )
    sampletest
    (
        .tri_R16S       (tri_R16S       ),
        .color_R16U     (color_R16U     ),
        .sample_R16S    (sample_R16S    ),
        .validSamp_R16H (validSamp_R16H ),

        .clk            (clk            ),
        .rst            (rst            ),

        .hit_R18S       (hit_R18S       ),
        .color_R18U     (color_R18U     ),
        .hit_valid_R18H (hit_valid_R18H )
    );

    // Generating vectors for sampletest, printed in the same order as the
    // signals in the magma module
    `ifdef GENERATE_JSON
    always @(posedge clk) begin
        $fdisplay(sample_file, "{\"RESET\": \"%b\",", rst);

        $fdisplay(sample_file, "\"tri\": [[\"%h\", \"%h\", \"%h\"], [\"%h\", \"%h\", \"%h\"], [\"%h\", \"%h\", \"%h\"]]",
        tri_R16S[0][0],
        tri_R16S[0][1],
        tri_R16S[0][2],
        tri_R16S[1][0],
        tri_R16S[1][1],
        tri_R16S[1][2],
        tri_R16S[2][0],
        tri_R16S[2][1],
        tri_R16S[2][2], 
        ",");

        $fdisplay(sample_file, "\"color_in\": [\"%h\", \"%h\", \"%h\"]",
        color_R16U[0],
        color_R16U[1],
        color_R16U[2],
        ",");

        $fdisplay(sample_file, "\"sample\": [\"%b\", \"%b\"]",
        sample_R16S[0],
        sample_R16S[1],
        ",");

        $fdisplay(sample_file, "\"valid_sample\": \"%b\",", validSamp_R16H);

        $fdisplay(sample_file, "\"hit\": [\"%h\", \"%h\", \"%h\"]",
        hit_R18S[0],
        hit_R18S[1],
        hit_R18S[2],
        ",");

        $fdisplay(sample_file, "\"valid_hit\": \"%b\",", hit_valid_R18H);

        $fdisplay(sample_file, "\"color_out\": [\"%h\", \"%h\", \"%h\"]",
        color_R18U[0],
        color_R18U[1],
        color_R18U[2]
        );

        $fdisplay(sample_file, "},");
    end

    final begin
        $fdisplay(bbox_file, "]");
        $fdisplay(sample_file, "]");
        $fdisplay(hash_file, "]");
        $fdisplay(iterator_file, "]");
        $fdisplay(rast_file, "]");
    end 
    `endif

endmodule
/*
 *  Performs Sample Test on triangle
 *
 *  Inputs:
 *    Sample and triangle Information
 *
 *  Outputs:
 *    Subsample Hit Flag, Subsample location, and triangle Information
 *
 *  Function:
 *    Utilizing Edge Equations determine whether the
 *    sample location lies inside the triangle.
 *    In the simple case of the triangle, this will
 *    occur when the sample lies to one side of all
 *    three lines (either all left or all right).
 *    This corresponds to the minterm 000 and 111.
 *    Additionally, if backface culling is performed,
 *    then only keep the case of all right.
 *
 *  Edge Equation:
 *    For an edge defined as travelling from the
 *    vertice (x_1,y_1) to (x_2,y_2), the sample
 *    (x_s,y_s) lies to the right of the line
 *    if the following expression is true:
 *
 *    0 >  ( x_2 - x_1 ) * ( y_s - y_1 ) - ( x_s - x_1 ) * ( y_2 - y_1 )
 *
 *    otherwise it lies on the line (exactly 0) or
 *    to the left of the line.
 *
 *    This block evaluates the six edges described by the
 *    triangles vertices,  to determine which
 *    side of the lines the sample point lies.  Then it
 *    determines if the sample point lies in the triangle
 *    by or'ing the appropriate minterms.  In the case of
 *    the triangle only three edges are relevant.  In the
 *    case of the quadrilateral five edges are relevant.
 *
 *
 *   Author: John Brunhaver
 *   Created:      Thu 07/23/09
 *   Last Updated: Tue 10/06/10
 *
 *   Copyright 2009 <jbrunhaver@gmail.com>
 *
 *
 */

/* A Note on Signal Names:
 *
 * Most signals have a suffix of the form _RxxxxN
 * where R indicates that it is a Raster Block signal
 * xxxx indicates the clock slice that it belongs to
 * and N indicates the type of signal that it is.
 * H indicates logic high, L indicates logic low,
 * U indicates unsigned fixed point, and S indicates
 * signed fixed point.
 *
 */

module sampletest
#(
    parameter SIGFIG        = 24, // Bits in color and position.
    parameter RADIX         = 10, // Fraction bits in color and position
    parameter VERTS         = 3, // Maximum Vertices in triangle
    parameter AXIS          = 3, // Number of axis foreach vertex 3 is (x,y,z).
    parameter COLORS        = 3, // Number of color channels
    parameter PIPE_DEPTH    = 2 // How many pipe stages are in this block
)
(
    input logic signed [SIGFIG-1:0]     tri_R16S[VERTS-1:0][AXIS-1:0], // triangle to Iterate Over
    input logic unsigned [SIGFIG-1:0]   color_R16U[COLORS-1:0] , // Color of triangle
    input logic signed [SIGFIG-1:0]     sample_R16S[1:0], // Sample Location
    input logic                         validSamp_R16H, // A valid sample location

    input logic clk, // Clock
    input logic rst, // Reset

    output logic signed [SIGFIG-1:0]    hit_R18S[AXIS-1:0], // Hit Location
    output logic unsigned [SIGFIG-1:0]  color_R18U[COLORS-1:0] , // Color of triangle
    output logic                        hit_valid_R18H                   // Is hit good
);

    localparam EDGES = (VERTS == 3) ? 3 : 5;
    //localparam SHORTSF = SIGFIG;
    localparam SHORTSF = SIGFIG-(RADIX-2);
    localparam MROUND = (2 * SHORTSF) - RADIX;

    // output for retiming registers
    logic signed [SIGFIG-1:0]       hit_R18S_retime[AXIS-1:0];   // Hit Location
    logic unsigned [SIGFIG-1:0]     color_R18U_retime[COLORS-1:0];   // Color of triangle
    logic                           hit_valid_R18H_retime;   // Is hit good
    // output for retiming registers

    // Signals in Access Order
    logic signed [SIGFIG-1:0]       tri_shift_R16S[VERTS-1:0][1:0]; // triangle after coordinate shift
    logic signed [SIGFIG-1:0]       edge_R16S[EDGES-1:0][1:0][1:0]; // Edges
    logic signed [(2*SHORTSF)-1:0]  dist_lg_R16S[EDGES-1:0]; // Result of x_1 * y_2 - x_2 * y_1
    logic                           hit_valid_R16H ; // Output (YOUR JOB!)
    logic signed [SIGFIG-1:0]       hit_R16S[AXIS-1:0]; // Sample position
    // Signals in Access Order

    // Your job is to produce the value for hit_valid_R16H signal, which indicates whether a sample lies inside the triangle.
    // hit_valid_R16H is high if validSamp_R16H && sample inside triangle (with back face culling)
    // Consider the following steps:

    // START CODE HERE
    // (1) Shift X, Y coordinates such that the fragment resides on the (0,0) position.
    // (2) Organize edges (form three edges for triangles)
    // (3) Calculate distance x_1 * y_2 - x_2 * y_1
    // (4) Check distance and assign hit_valid_R16H.
    always_comb begin
        //(1) Shift X, Y coordinates such that the fragment resides on the (0,0) position.
        tri_shift_R16S[0][0] = tri_R16S[0][0] - sample_R16S[0]; //v0, x
        tri_shift_R16S[1][0] = tri_R16S[1][0] - sample_R16S[0]; //v1, x
        tri_shift_R16S[2][0] = tri_R16S[2][0] - sample_R16S[0]; //v2, x

        tri_shift_R16S[0][1] = tri_R16S[0][1] - sample_R16S[1]; //v0, y
        tri_shift_R16S[1][1] = tri_R16S[1][1] - sample_R16S[1]; //v1, y
        tri_shift_R16S[2][1] = tri_R16S[2][1] - sample_R16S[1]; //v2, y

        // (3) Calculate distance x_1 * y_2 - x_2 * y_1
        dist_lg_R16S[0] = tri_shift_R16S[0][0]*tri_shift_R16S[1][1] - tri_shift_R16S[1][0]*tri_shift_R16S[0][1]; //e0_dist
        dist_lg_R16S[1] = tri_shift_R16S[1][0]*tri_shift_R16S[2][1] - tri_shift_R16S[2][0]*tri_shift_R16S[1][1]; //e1_dist
        dist_lg_R16S[2] = tri_shift_R16S[2][0]*tri_shift_R16S[0][1]- tri_shift_R16S[0][0]*tri_shift_R16S[2][1]; //e0_dist

        // (4) Check distance and assign hit_valid_R16H.
        hit_valid_R16H = (dist_lg_R16S[0] <= 0) && (dist_lg_R16S[1] < 0) && (dist_lg_R16S[2] <= 0);
    end 

    // END CODE HERE

    //Assertions to help debug
    //Check if correct inequalities have been used
    assert property( @(posedge clk) (dist_lg_R16S[1] == 0) |-> !hit_valid_R16H);

    //Calculate Depth as depth of first vertex
    // Note that a barrycentric interpolation would
    // be more accurate
    always_comb begin
        hit_R16S[1:0] = sample_R16S[1:0]; //Make sure you use unjittered sample
        hit_R16S[2] = tri_R16S[0][2]; // z value equals the z value of the first vertex
    end

    /* Flop R16 to R18_retime with retiming registers*/
    dff2 #(
        .WIDTH          (SIGFIG         ),
        .ARRAY_SIZE     (AXIS           ),
        .PIPE_DEPTH     (PIPE_DEPTH - 1 ),
        .RETIME_STATUS  (1              )
    )
    d_samp_r1
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (hit_R16S       ),
        .out    (hit_R18S_retime)
    );

    dff2 #(
        .WIDTH          (SIGFIG         ),
        .ARRAY_SIZE     (COLORS         ),
        .PIPE_DEPTH     (PIPE_DEPTH - 1 ),
        .RETIME_STATUS  (1              )
    )
    d_samp_r2
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (1'b1               ),
        .in     (color_R16U         ),
        .out    (color_R18U_retime  )
    );

    dff_retime #(
        .WIDTH          (1              ),
        .PIPE_DEPTH     (PIPE_DEPTH - 1 ),
        .RETIME_STATUS  (1              ) // RETIME
    )
    d_samp_r3
    (
        .clk    (clk                    ),
        .reset  (rst                    ),
        .en     (1'b1                   ),
        .in     (hit_valid_R16H         ),
        .out    (hit_valid_R18H_retime  )
    );
    /* Flop R16 to R18_retime with retiming registers*/

    /* Flop R18_retime to R18 with fixed registers */
    dff2 #(
        .WIDTH          (SIGFIG ),
        .ARRAY_SIZE     (AXIS   ),
        .PIPE_DEPTH     (1      ),
        .RETIME_STATUS  (0      )
    )
    d_samp_f1
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (hit_R18S_retime),
        .out    (hit_R18S       )
    );

    dff2 #(
        .WIDTH          (SIGFIG ),
        .ARRAY_SIZE     (COLORS ),
        .PIPE_DEPTH     (1      ),
        .RETIME_STATUS  (0      )
    )
    d_samp_f2
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (1'b1               ),
        .in     (color_R18U_retime  ),
        .out    (color_R18U         )
    );

    dff #(
        .WIDTH          (1  ),
        .PIPE_DEPTH     (1  ),
        .RETIME_STATUS  (0  ) // No retime
    )
    d_samp_f3
    (
        .clk    (clk                    ),
        .reset  (rst                    ),
        .en     (1'b1                   ),
        .in     (hit_valid_R18H_retime  ),
        .out    (hit_valid_R18H         )
    );

    /* Flop R18_retime to R18 with fixed registers */

endmodule/*
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
    logic signed [SIGFIG-1:0]   box_R14S[1:0][1:0];    		// the state for current bounding box
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

    //////
    ////// First calculate the values of the helper signals using CURRENT STATES
    //////

    // check the comments 'A Note on Signal Names'
    // at the begining of the module for the help on
    // understanding the signals here

    always_comb begin
        // START CODE HERE
        unique case(1'b1) //REVERSE CASE STATEMENT FOR ONE-HOT SIGNALS // synopsys full_case
            subSample_RnnnnU[0]: begin //0001
                next_rt_samp_R14S[1] = sample_R14S[1]; //ll_y co-ord of sample location to be tested 
                next_up_samp_R14S[0] = box_R14S[0][0]; //x co-ord of current bbox

                next_rt_samp_R14S[0][SIGFIG-1:RADIX-3] = sample_R14S[0][SIGFIG-1:RADIX-3] + 1'b1;
                next_rt_samp_R14S[0][RADIX-4:0] = sample_R14S[0][RADIX-4:0];
                next_up_samp_R14S[1][SIGFIG-1:RADIX-3] = sample_R14S[1][SIGFIG-1:RADIX-3] + 1'b1;
                next_up_samp_R14S[1][RADIX-4:0] = sample_R14S[1][RADIX-4:0];
            end
            subSample_RnnnnU[1]: begin //0010
                next_rt_samp_R14S[1] = sample_R14S[1]; 
                next_up_samp_R14S[0] = box_R14S[0][0];
                
                next_rt_samp_R14S[0][SIGFIG-1:RADIX-2] = sample_R14S[0][SIGFIG-1:RADIX-2] + 1'b1;
                next_rt_samp_R14S[0][RADIX-3:0] = sample_R14S[0][RADIX-3:0];
                next_up_samp_R14S[1][SIGFIG-1:RADIX-2] = sample_R14S[1][SIGFIG-1:RADIX-2] + 1'b1;
                next_up_samp_R14S[1][RADIX-3:0] = sample_R14S[1][RADIX-3:0];
            end
            subSample_RnnnnU[2]: begin //0100
                next_rt_samp_R14S[1] = sample_R14S[1]; 
                next_up_samp_R14S[0] = box_R14S[0][0];
                
                next_rt_samp_R14S[0][SIGFIG-1:RADIX-1]= sample_R14S[0][SIGFIG-1:RADIX-1] + 1'b1;
                next_rt_samp_R14S[0][RADIX-2:0] = sample_R14S[0][RADIX-2:0];
                next_up_samp_R14S[1][SIGFIG-1:RADIX-1] = sample_R14S[1][SIGFIG-1:RADIX-1] + 1'b1;
                next_up_samp_R14S[1][RADIX-2:0] = sample_R14S[1][RADIX-2:0];
            end
            subSample_RnnnnU[3]: begin //1000
                next_rt_samp_R14S[1] = sample_R14S[1]; 
                next_up_samp_R14S[0] = box_R14S[0][0];
                
                next_rt_samp_R14S[0][SIGFIG-1:RADIX]= sample_R14S[0][SIGFIG-1:RADIX] + 1'b1;
                next_rt_samp_R14S[0][RADIX-1:0] = sample_R14S[0][RADIX-1:0];
                next_up_samp_R14S[1][SIGFIG-1:RADIX] = sample_R14S[1][SIGFIG-1:RADIX] + 1'b1;
                next_up_samp_R14S[1][RADIX-1:0] = sample_R14S[1][RADIX-1:0];
            end
        endcase

        at_right_edg_R14H = (sample_R14S[0] == box_R14S[1][0]);
        at_top_edg_R14H = (sample_R14S[1] == box_R14S[1][1]);
        at_end_box_R14H = (sample_R14S == box_R14S[1]);
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
        case(state_R14H)
            WAIT_STATE : begin
                next_box_R14S = box_R13S;
                next_tri_R14S = tri_R13S;
                next_color_R14U = color_R13U;
                next_sample_R14S = box_R13S[0]; //ll bbox
                next_validSamp_R14H = validTri_R13H ? 1'b1 :1'b0;
                next_halt_RnnnnL = validTri_R13H ? 1'b0 : 1'b1; 
                next_state_R14H = validTri_R13H ? TEST_STATE : WAIT_STATE;
            end
            TEST_STATE : begin
                next_state_R14H = (at_end_box_R14H ? WAIT_STATE : TEST_STATE);  
                next_box_R14S = box_R14S;
                next_tri_R14S = tri_R14S; 
                next_color_R14U = color_R14U;                  
                unique case(1'b1) // synopsys full_case
                    !at_right_edg_R14H & !at_end_box_R14H: begin
                        //if (!at_right_edg_R14H && !at_end_box_R14H) begin
                        next_sample_R14S = next_rt_samp_R14S;
                        next_validSamp_R14H = 1'b1;
                        next_halt_RnnnnL = 1'b0;
                        
                    end
                    !at_end_box_R14H & at_right_edg_R14H: begin
                    //else if (!at_end_box_R14H && at_right_edg_R14H) begin
                        next_sample_R14S = next_up_samp_R14S;
                        next_validSamp_R14H = 1'b1;
                        next_halt_RnnnnL = 1'b0;
                        end
                    at_end_box_R14H: begin
                    //else begin
                    next_sample_R14S = box_R14S[0]; //ll bbox
                    next_validSamp_R14H = 1'b0;
                    next_halt_RnnnnL = 1'b1;
                    end
                endcase
            end
        endcase
        // END CODE HERE
    end // always_comb

    //Assertions for testing FSM logic

    // Write assertions to verify your FSM transition sequence
    // Can you verify that:
    // 1) A validTri_R13H signal causes a transition from WAIT state to TEST state
    // 2) An end_box_R14H signal causes a transition from TEST state to WAIT state
    // 3) What are you missing?

    //Your assertions goes here
    // START CODE HERE
    assert property (@(posedge clk) validTri_R13H && state_R14H==WAIT_STATE |-> next_state_R14H==TEST_STATE);
    assert property (@(posedge clk) at_end_box_R14H && state_R14H==TEST_STATE |-> next_state_R14H==WAIT_STATE);
    assert property (@(posedge clk) next_halt_RnnnnL && state_R14H==TEST_STATE |-> next_state_R14H==WAIT_STATE);
    assert property (@(posedge clk) next_halt_RnnnnL && state_R14H==WAIT_STATE |=> state_R14H==WAIT_STATE);
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
    assert property (rb_lt(rst, next_sample_R14S[0], next_box_R14S[1][0], next_validSamp_R14H)); //ns_x <= nb_ur_x
    assert property (rb_lt(rst, next_sample_R14S[1], next_box_R14S[1][1], next_validSamp_R14H)); //ns_y <= nb_ur_y
    assert property (rb_lt(rst, next_box_R14S[0][0], next_sample_R14S[0], next_validSamp_R14H)); //nb_ll_x <= ns_x
    assert property (rb_lt(rst, next_box_R14S[0][1], next_sample_R14S[1], next_validSamp_R14H)); //nb_ur_y <= ns_y
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
/*
 *  Hashing Function
 *
 *  Inputs:
 *    N-Wide Signal
 *
 *  Outputs:
 *    M-Bit Hashed signal
 *
 *  Function:
 *    Calc a simple hash value useing an xor tree
 *
 *
 *
 *   Author: John Brunhaver
 *   Created:      Thu 10/01/10
 *   Last Updated: Tue 10/16/10
 *
 *   Copyright 2010 <jbrunhaver@gmail.com>
 *
 */

/* ***************************************************************************
 * Change bar:
 * -----------
 * Date           Author    Description
 * Sep 19, 2012   jingpu    ported from John's original code to Genesis
 *
 * ***************************************************************************/

/******************************************************************************
 * PARAMETERIZATION
 * ***************************************************************************/
//; # module parameters
//; my $in_width   = parameter(Name=>'InWidth',
//;                            Val=>40, Min=>40, Step=>1, Max=>40,
//;                            Doc=>"Width of Input");
//; my $out_width  = parameter(Name=>'OutWidth',
//;                            Val=>8, Min=>8, Step=>1, Max=>8,
//;                            Doc=>"Width of output");
//; # Note that these are not yet configurable
//; # this module depends on these statements being 40,8
//; # note that it is possible to build a recursive version
//; # of this module which can generically build a hash tree
//; # for arbitrary N and M.
//; # General strategy:
//; #   *Reduce input to a width that (2^n) * Output Width
//; #   *Swizel and reduce to 2^(n-1) and repeat

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
 */

module tree_hash
#(
    parameter IN_WIDTH = 40,
    parameter OUT_WIDTH = 8
)
(
    // Input Signals
    input logic unsigned [IN_WIDTH-1:0]      in_RnnH, //Input signal to derive hash from
    input logic unsigned [OUT_WIDTH-1:0]     mask_RnnH, //A mask to apply to the hashed output
    // Output Signals
    output logic unsigned [OUT_WIDTH-1:0]    out_RnnH   //Output signal that has been hashed and masked
);

    logic unsigned [31:0]   arr32_RnnH ;
    logic unsigned [15:0]   arr16_RnnH ;

    // IN_WIDTH that this is brittle and will break for any config that isn't 40:8
    assign arr32_RnnH[7:0]   = in_RnnH[7:0]   ^ in_RnnH[15:8]  ; // 0 = 0 ^ 1
    assign arr32_RnnH[15:8]  = in_RnnH[15:8]  ^ in_RnnH[23:16] ; // 1 = 1 ^ 2
    assign arr32_RnnH[23:16] = in_RnnH[23:16] ^ in_RnnH[31:24] ; // 2 = 2 ^ 3
    assign arr32_RnnH[31:24] = in_RnnH[31:24] ^ in_RnnH[39:32] ; // 3 = 3 ^ 4

    assign arr16_RnnH[7:0] = arr32_RnnH[7:0] ^ arr32_RnnH[23:16] ; // 0 = 0 ^ 2
    assign arr16_RnnH[15:8] = arr32_RnnH[15:8] ^ arr32_RnnH[31:24] ; // 1 ^ 3

    assign out_RnnH[OUT_WIDTH-1:0] = ( arr16_RnnH[7:0] ^ arr16_RnnH[15:8] ) & mask_RnnH[7:0] ;

endmodule



////////////////////////////////////////////////////////////////////////////////
//
//       This confidential and proprietary software may be used only
//     as authorized by a licensing agreement from Synopsys Inc.
//     In the event of publication, the following notice is applicable:
//
//                    (C) COPYRIGHT 2007 - 2017 SYNOPSYS INC.
//                           ALL RIGHTS RESERVED
//
//       The entire notice above must be reproduced on all authorized
//     copies.
//
// AUTHOR:    Rick Kelly      May 2, 2007
//
// VERSION:   Verilog Simulation Model
//
// DesignWare_version: c17dcf7a
// DesignWare_release: M-2016.12-DWBB_201612.2
//
////////////////////////////////////////////////////////////////////////////////
//
// ABSTRACT: Pipeline register with parameter control for width, pipe stages
//		as well as non-retimable input or output register
//
//		Register are individually enabled by separate bits of the
//		enable input bus.
//
//
//              Parameters:     Valid Values
//              ==========      ============
//              width           [ > 0 ]
//              in_reg          [ 0 = no fixed input register
//				  1 = fixed (not retimable) input register ]
//              stages          [ > 0 ]
//              out_reg         [ 0 = no fixed output register
//				  1 = fixed (not retimable) output register ]
//              rst_mode        [ 0 = asynchronous reset,
//                                1 = synchronous reset ]
//		
//		Input Ports:	Size	Description
//		===========	====	===========
//		clk		1 bit	Input Clock
//		rst_n		1 bit	Active Low Reset
//		enable	       EW bits	Active High Enable Bus
//		data_in		width	Data input port
//
//		Output Ports	Size	Description
//		============	====	===========
//		data_out	width	Data output port
//
//	where :  EW = min(1, in_reg + stages + out_reg - 1)
//
// MODIFIED: 
//
//      RJK 01/10/13  Updated coding to use standard sequential block
//                    coding without intermediate signals (using V2K
//                    generate blocks to differentiate async from sync
//                    reset modes).  (STAR 9000589609)
//
////////////////////////////////////////////////////////////////////////////////-

module DW_pl_reg ( clk, rst_n, enable,
		    data_in, data_out);

parameter width = 8;	// NATURAL
parameter in_reg = 0;   // RANGE 0 to 1
parameter stages = 4;	// NATURAL
parameter out_reg = 0;  // RANGE 0 to 1
parameter rst_mode = 0;	// RANGE 0 to 1

localparam en_msb = (stages-1+in_reg+out_reg < 1)? 0 : (stages+in_reg+out_reg-2);

input			clk;		// clock input
input			rst_n;		// active low reset input
input  [en_msb : 0]	enable;		// active high enable input bus
input  [width-1 : 0]	data_in;	// input data bus

output [width-1 : 0]	data_out;	// output data bus
   
reg    [width-1 : 0]	pipe_regs [0 : en_msb];



generate
 if (rst_mode == 0) begin : REG1_ASYNC_RST
  always @ (posedge clk or negedge rst_n) begin : PROC_registers
    integer i;

    if (rst_n === 1'b0) begin
      for (i=0 ; i <= en_msb ; i=i+1) begin
	pipe_regs[i] <= {width{1'b0}};
      end
    end else if (rst_n === 1'b1) begin
      for (i=0 ; i <= en_msb ; i=i+1) begin
        if (enable[i] === 1'b1)
	  pipe_regs[i] <= (i == 0)? (data_in | (data_in ^ data_in)) : pipe_regs[i-1];
	else if (enable[i] !== 1'b0)
	  pipe_regs[i] <= ((pipe_regs[i] ^ ((i == 0)? (data_in | (data_in ^ data_in)) : pipe_regs[i-1]))
			      & {width{1'bx}}) ^ pipe_regs[i];
      end
    end else begin
      for (i=0 ; i <= en_msb ; i=i+1) begin
	pipe_regs[i] <= {width{1'bx}};
      end
    end
  end
 end else begin : REG1_SYNC_RST
  always @ (posedge clk) begin : PROC_registers
    integer i;

    if (rst_n === 1'b0) begin
      for (i=0 ; i <= en_msb ; i=i+1) begin
	pipe_regs[i] <= {width{1'b0}};
      end
    end else if (rst_n === 1'b1) begin
      for (i=0 ; i <= en_msb ; i=i+1) begin
        if (enable[i] === 1'b1)
	  pipe_regs[i] <= (i == 0)? (data_in | (data_in ^ data_in)) : pipe_regs[i-1];
	else if (enable[i] !== 1'b0)
	  pipe_regs[i] <= ((pipe_regs[i] ^ ((i == 0)? (data_in | (data_in ^ data_in)) : pipe_regs[i-1]))
			      & {width{1'bx}}) ^ pipe_regs[i];
      end
    end else begin
      for (i=0 ; i <= en_msb ; i=i+1) begin
	pipe_regs[i] <= {width{1'bx}};
      end
    end
  end
 end
endgenerate


  assign data_out = (in_reg+stages+out_reg == 1)? (data_in | (data_in ^ data_in)) : pipe_regs[en_msb];

endmodule
