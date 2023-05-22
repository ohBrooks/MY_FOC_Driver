SamacSys ECAD Model
556575/869330/2.50/2/3/Diode ESD Uni-directional

DESIGNSPARK_INTERMEDIATE_ASCII

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r220_155"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 1.55) (shapeHeight 2.2))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(textStyleDef "Default"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 50 mils)
			(strokeWidth 5 mils)
		)
	)
	(patternDef "DIOM5127X229N" (originalName "DIOM5127X229N")
		(multiLayer
			(pad (padNum 1) (padStyleRef r220_155) (pt -2.05, 0) (rotation 90))
			(pad (padNum 2) (padStyleRef r220_155) (pt 2.05, 0) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Default") (isVisible True))
		)
		(layerContents (layerNumRef 30)
			(line (pt -3.4 1.645) (pt 3.4 1.645) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt 3.4 1.645) (pt 3.4 -1.645) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt 3.4 -1.645) (pt -3.4 -1.645) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt -3.4 -1.645) (pt -3.4 1.645) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.552 1.332) (pt 2.552 1.332) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 2.552 1.332) (pt 2.552 -1.332) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 2.552 -1.332) (pt -2.552 -1.332) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.552 -1.332) (pt -2.552 1.332) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.552 0.558) (pt -1.778 1.332) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt 2.552 1.332) (pt -2.825 1.332) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.552 -1.332) (pt 2.552 -1.332) (width 0.2))
		)
	)
	(symbolDef "SMAJ12A" (originalName "SMAJ12A")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 100 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 140 mils -15 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 2) (pt 600 mils 0 mils) (rotation 180) (pinLength 100 mils) (pinDisplay (dispPinName false)) (pinName (text (pt 460 mils -15 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(line (pt 200 mils 0 mils) (pt 400 mils 100 mils) (width 6 mils))
		(line (pt 400 mils 100 mils) (pt 400 mils -100 mils) (width 6 mils))
		(line (pt 400 mils -100 mils) (pt 200 mils 0 mils) (width 6 mils))
		(line (pt 200 mils 100 mils) (pt 200 mils -100 mils) (width 6 mils))
		(line (pt 100 mils 0 mils) (pt 200 mils 0 mils) (width 6 mils))
		(line (pt 500 mils 0 mils) (pt 400 mils 0 mils) (width 6 mils))
		(line (pt 200 mils -100 mils) (pt 260 mils -100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 400 mils 350 mils) (justify Left) (isVisible True) (textStyleRef "Default"))

	)
	(compDef "SMAJ12A" (originalName "SMAJ12A") (compHeader (numPins 2) (numParts 1) (refDesPrefix D)
		)
		(compPin "1" (pinName "K") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "2" (pinName "A") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "SMAJ12A"))
		(attachedPattern (patternNum 1) (patternName "DIOM5127X229N")
			(numPads 2)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
			)
		)
		(attr "Manufacturer_Name" "LITTELFUSE")
		(attr "Manufacturer_Part_Number" "SMAJ12A")
		(attr "Mouser Part Number" "576-SMAJ12A")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Littelfuse/SMAJ12A?qs=HR2RnyOI4E63xEASay2NCQ%3D%3D")
		(attr "Arrow Part Number" "SMAJ12A")
		(attr "Arrow Price/Stock" "https://www.arrow.com/en/products/smaj12a/littelfuse?region=nac")
		(attr "Description" "LITTELFUSE - SMAJ12A - TVS Diode, SMAJ1 Series, Unidirectional, 12 V, 19.9 V, DO-214AC (SMA), 2 Pins")
		(attr "Datasheet Link" "https://www.littelfuse.com/~/media/electronics/datasheets/tvs_diodes/littelfuse_tvs_diode_smaj_datasheet.pdf.pdf")
		(attr "Height" "2.29 mm")
	)

)
