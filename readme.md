# Edge computing weather station - WORK IN PROGRESS

Platform: STM32F411 ("black pill" board)

The idea is to have temperature and humidity read from a sensor
and based on that classify the weather as good, bad or so-so.

The equations for decision boundaries between these classes were defined
using logistic regression from the scikit.learn ML package.

The parameters will be modified with further improvements of the model.
 
Dataset is purely synthetic, labelled according to some simple rules with mild randomization.
 
At the moment readings from the sensor are simulated with trimpots read by the ADC.


Current problems: 
* cannot use BME280 sensor as its driver requires BlockingI2C, not available in the STMF4xx HAL crate
* the decision boundaries are linear, working on a model with polynomial boundaries