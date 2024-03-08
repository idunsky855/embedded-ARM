# Embedded - ARM project

<p>
  A project based on STM32L552ZETQ board ( ARM processor ).
  The project has 4 states and everything that happens is based on timer interrupts, and cycling through the states happens with User Button interrupts.
</p>

## States:
<ol>
  <li>1. Regular traffic light simulation using LED lights red - red, blue - orange, green - green.</li>
  <li>2. Formula 1 like traffic light - left light on, left and middle lights on, all three lights on, lights off - go!</li>
  <li>3. Binary counter - using 3 lights counting 0-7.</li>
  <li>4. Crosswlak light - Stop - red, Walk - green.</li>
</ol>

