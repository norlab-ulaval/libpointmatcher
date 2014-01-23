$(document).ready(function() {
	// Every image referenced from a Markdown document
	$(".markdown img").each(function() {
		// Let's put a caption if there is one
		if($(this).attr("alt"))
			$(this).wrap('<figure class="image"></figure>')
				.after('<figcaption>'+$(this).attr("alt")+'</figcaption>');
		});
});
