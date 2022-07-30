default: xcode

xcode: 
	cmake -B build -G Xcode
	@open build/CG.xcodeproj


