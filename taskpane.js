/*
 * Copyright (c) Microsoft Corporation. All rights reserved. Licensed under the MIT license.
 * See LICENSE in the project root for license information.
 */

// images references in the manifest
import "../../assets/icon-16.png";
import "../../assets/icon-32.png";
import "../../assets/icon-80.png";

/* global console, document, Excel, Office */

Office.onReady(info => {
  if (info.host === Office.HostType.Excel) {
    // Determine if the user's version of Office supports all the Office.js APIs that are used in the tutorial.
    if (!Office.context.requirements.isSetSupported('ExcelApi', '1.7')) {
      console.log('Sorry. This add-in uses Excel.js APIs that are not available in your version of Office.');
    }
    // Assign event handlers and other initialization logic.
    document.getElementById("update-graph").onclick = graphResults;
    document.getElementById("sideload-msg").style.display = "none";
    document.getElementById("app-body").style.display = "flex";

  }
});

//organizes the data based on which periodicity is chosen
function graphResults() {
  Excel.run(function (context) {
    var currentWorksheet = context.workbook.worksheets.getActiveWorksheet();

      //turn cells that exceed SPL red
      //const allData = currentWorksheet.getRange("A:B");
      const SPLdata = currentWorksheet.getRange("C:C");
      SPLdata.conditionalFormats.clearAll();
      const conditionalFormat = SPLdata.conditionalFormats.add(Excel.ConditionalFormatType.cellValue);
      conditionalFormat.cellValue.format.fill.color = "#ff8080";
      var thrstr = "=" + `${document.getElementById("thr").value}`;
      if(document.getElementById("lower").checked){
        conditionalFormat.cellValue.rule = { formula1: thrstr, formula2:"=1", operator: "Between" };
      }else{
        conditionalFormat.cellValue.rule = { formula1: thrstr, formula2:"=500", operator: "Between" };
      }
      

      //pivot table
       var pivotTable = currentWorksheet.pivotTables.add("SPL Sorted Data","A:C","E1");
       pivotTable.rowHierarchies.add(pivotTable.hierarchies.getItem("Date"));
       pivotTable.rowHierarchies.add(pivotTable.hierarchies.getItem("Time"));
       pivotTable.dataHierarchies.add(pivotTable.hierarchies.getItem("SPL"));
       pivotTable.dataHierarchies.add(pivotTable.hierarchies.getItem("SPL"));
       pivotTable.dataHierarchies.add(pivotTable.hierarchies.getItem("SPL"));

       pivotTable.dataHierarchies.load("no-properties-needed");
       pivotTable.layoutType = "Compact";

      //create chart w threshold
      var numRange = currentWorksheet.getRange("E:F");
      var chart = currentWorksheet.charts.add("Line",numRange,"auto");
      chart.title.text = "SPL Weekly Report";
      chart.axes.categoryAxis.title.text = "Date";
      chart.axes.valueAxis.title.text = "SPL (dB)";
      chart.axes.valueAxis.minimum = 40;
      //chart.legend.visible = false;
             
       return context.sync().then(function() {
           // Change the aggregation from the default sum to an average of all the values in the hierarchy.
           pivotTable.dataHierarchies.items[0].summarizeBy = Excel.AggregationFunction.average;
           pivotTable.dataHierarchies.items[0].name = "Mean SPL";

           pivotTable.dataHierarchies.items[1].summarizeBy = Excel.AggregationFunction.min;
           pivotTable.dataHierarchies.items[1].name = "Min SPL";

           pivotTable.dataHierarchies.items[2].summarizeBy = Excel.AggregationFunction.max;
           pivotTable.dataHierarchies.items[2].name = "Max SPL";

           return context.sync();
       });
      
  })
  .catch(function (error) {
      console.log("Error: " + error);
      if (error instanceof OfficeExtension.Error) {
          console.log("Debug info: " + JSON.stringify(error.debugInfo));
      }
  });
}